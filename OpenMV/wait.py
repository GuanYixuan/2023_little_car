import time
from pyb import UART

WAIT_TIME_MAX = 2
uart = UART(3, 9600)

def wait_uart(change_times, uart_time):
    """
    判断前一次指令是否执行完
    """
    if change_times == 1:
        print("uart write angle 1")
        return True
    else:
        if uart.any():
            command_receive = uart.readline().decode().strip()
            print(command_receive)
            if "steer" in command_receive or "shift" in command_receive:
                print("uart 接收返回信号")
                return True
        else:
            delta_time = time.time() - uart_time
            if delta_time > WAIT_TIME_MAX:
                print("uart 信号超时")
                return True
    return False
