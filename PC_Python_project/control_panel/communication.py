"""根据通讯协议及STM32指令格式, 提供了一个封装过的串口对象`Serial_handler`"""

import time
import serial
import threading
from multiprocessing.queues import Queue

from typing import List

STM32_INSTRUCTION_LENGTH: int = 8
PING_ECHO_CONTENT: str = "PINECO\n"
GRAB_SUCCESS_CONTENT: str = "GRSUCC"
GRAB_FAILED_CONTENT: str = "GRFAIL"
PLACE_SUCCESS_CONTENT: str = "PLSUCC"
PLACE_FAILED_CONTENT: str = "PLFAIL"

class Serial_handler:
    """用于PC-STM32交互的串口对象"""

    running: bool

    serial_instance: serial.Serial
    """被封装的底层串口对象"""

    instruction_queue: List[bytes]
    """等待发出的指令"""
    message_queue: List[bytes]
    """从STM32收到的信息, 结尾的'\\n'已经去除"""

    feeder_thread: threading.Thread
    """指令发送线程"""
    receiver_thread: threading.Thread
    """指令接收线程"""

    ping_cond: threading.Condition
    """用于ping指令的同步变量"""
    recv_cond: threading.Condition
    """用于接收信息的同步变量"""

    extra_queues: List[Queue]
    """附加的消息队列, 每收到一条消息都会向其中的所有队列抄送一份"""

    def __init__(self, serial_name: str, baudrate: int) -> None:
        # 属性初始化
        self.running = True
        self.message_queue = []
        self.extra_queues = []
        self.instruction_queue = []
        self.serial_instance = serial.Serial(serial_name, baudrate)
        self.serial_instance.timeout = 0.1

        # 各线程初始化
        self.ping_cond = threading.Condition()
        self.recv_cond = threading.Condition()
        self.feeder_thread = threading.Thread(target=self.__instruction_feeder, name="Serial_handler-instruction_feeder", daemon=True)
        self.receiver_thread = threading.Thread(target=self.__message_receiver, name="Serial_handler-message_receiver", daemon=True)
        self.feeder_thread.start()
        self.receiver_thread.start()

    def deinit(self) -> None:
        """析构函数"""
        self.running = False
        self.feeder_thread.join()
        self.receiver_thread.join()

    def inst_steer(self, angle: int) -> None:
        """非阻塞地向小车发送转向指令

        Args:
            `angle` (int): 逆时针转动的角度数
        """
        self.instruction_queue.append(bytes([0x10]) + angle.to_bytes(2, 'little', signed=True) + bytes([0 for i in range(5)]))
    def inst_shift(self, forward: int, shift_right: int) -> None:
        """非阻塞地向小车发送平移指令

        Args:
            `forward` (int): 小车前进的毫米数
            `shift_right` (int): 小车向右平移的毫米数
        """
        self.instruction_queue.append(bytes([0x20]) + forward.to_bytes(2, 'little', signed=True) + shift_right.to_bytes(2, 'little', signed=True) + bytes([0, 0, 0]))
    def inst_echo(self, content: str) -> None:
        """非阻塞地向STM32发送回显指令

        Args:
            `content` (str): 要向回显的信息, 注意不能超过7字节

        Raises:
            `AssertionError`: 要发送的信息包含非ASCII字节或过长
        """
        assert len(content) <= STM32_INSTRUCTION_LENGTH - 1 and content.isascii()
        padding = bytes([0 for i in range(STM32_INSTRUCTION_LENGTH - 1 - len(content))])

        self.instruction_queue.append(bytes([0x31]) + content.encode() + padding)
    def inst_to_OpenMV(self, content: bytes) -> None:
        """非阻塞地向OpenMV发送信息, 该信息会通过STM32进行转发

        Args:
            `content` (bytes): 要向OpenMV发送的信息, 注意不能超过7字节

        Raises:
            `AssertionError`: 要发送的信息过长
        """
        assert len(content) <= STM32_INSTRUCTION_LENGTH - 1, "要发送的消息过长"
        if len(content) < STM32_INSTRUCTION_LENGTH - 1:
            content += bytes([0 for i in range(STM32_INSTRUCTION_LENGTH - 1 - len(content))])

        self.instruction_queue.append(bytes([0x32]) + content)
    def inst_grab_mode(self) -> None:
        """非阻塞地向小车发送'进入抓取状态'指令"""
        self.inst_to_OpenMV("GRASP \n".encode())
    def inst_place_mode(self) -> None:
        """非阻塞地向小车发送'进入放置状态'指令"""
        self.inst_to_OpenMV("PLACE \n".encode())

    def ping_STM32(self, timeout: float) -> float:
        """阻塞式地ping STM32, 返回延迟时间"""
        self.inst_echo(PING_ECHO_CONTENT)
        start_time = time.monotonic()

        with self.ping_cond:
            self.ping_cond.wait(timeout)
        return time.monotonic() - start_time

    def __instruction_feeder(self) -> None:
        """指令发送线程"""
        while self.running:
            if len(self.instruction_queue):
                self.serial_instance.write(self.instruction_queue[0])
                self.instruction_queue.pop(0)
    def __message_receiver(self) -> None:
        """信息接收线程"""
        buffer = bytes()
        while self.running:
            new_bytes = self.serial_instance.read_all()
            assert isinstance(new_bytes, bytes)
            buffer += new_bytes

            slice_position = buffer.find(bytes([ord('\n')]))
            if slice_position < 0:
                continue
            new_message = buffer[:slice_position]

            # 区分PING_ECHO包并提前处理
            need_append: bool = True
            try:
                if new_message.decode().upper() == PING_ECHO_CONTENT[:-1]:
                    need_append = False
                    with self.ping_cond:
                        self.ping_cond.notify_all()
            except UnicodeDecodeError:
                pass

            # 将消息加入队列
            if need_append:
                self.message_queue.append(new_message)
                for queue in self.extra_queues:
                    queue.put(new_message)

                # 通知各个正在等待的线程
                with self.recv_cond:
                    self.recv_cond.notify_all()

            buffer = buffer[slice_position+1:]

if __name__ == "__main__":
    ser = Serial_handler("COM9", 9600)

    try:
        while True:
            print("%.2f" % ser.ping_STM32(2))
            if len(ser.message_queue):
                print(ser.message_queue.pop(0).decode())
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    ser.deinit()
