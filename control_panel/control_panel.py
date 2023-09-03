"""小车的控制面板"""

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import math
import time
import threading
import numpy as np
import multiprocessing
from multiprocessing.queues import Queue
from multiprocessing.synchronize import Condition

from ui_main_window import Ui_MainWindow
from PyQt6.QtCore import pyqtSignal
from qtpy.QtWidgets import QMainWindow, QApplication
from qtpy.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QScrollBar
from qtpy.QtGui import QPixmap, QImage, QTextCharFormat, QColor, QTextCursor

# 从自己写的包中import
from communication import Serial_handler
from camera.camera import Camera, Camera_message
import camera.constants as CC # camera constants

# 类型注释系列import
from typing import Optional
from numpy.typing import NDArray

BAUDRATE: int = 9600
SERIAL_NAME: str = "COM9"

STEER_STEP_LENGTH: int = 20
SHIFT_STEP_LENGTH: int = 30

COMMAND_COLOR: int = 0x000000
SUCCESS_COLOR: int = 0x009000
FAILED_COLOR: int = 0xa00000

def camera_process(cond: Optional[Condition] = None, queue: Optional[Queue] = None) -> None:
    cam = Camera(cond=cond, queue=queue)
    cam.main_loop()

class Control_panel(QMainWindow, Ui_MainWindow):
    """小车控制面板"""

    running: bool = True
    no_STM: bool
    no_camera: bool

    stm32_serial: Serial_handler
    """STM32串口管理对象"""
    camera_cond: Condition
    camera_queue: Queue
    camera_message: Camera_message
    """最新的场外相机信息"""

    log_panel_thread: threading.Thread
    camera_process: multiprocessing.Process
    camera_listener: threading.Thread
    camera_signal = pyqtSignal(str) # 不知为何必须是class member

    def __init__(self, *, no_STM: bool = False, no_camera: bool = False) -> None:
        """启动小车控制面板

        Args:
            `no_STM` (bool, optional): 是否在不连接小车的情况下启动, 默认为否. 此时无法使用控制面板发送指令
            `no_camera` (bool, optional): 是否在不连接相机的情况下启动, 默认为否. 此时无法在控制面板中看到场地信息及定位信息.
        """
        # 启动GUI
        super().__init__(None)
        self.setupUi(self)

        # 初始化各属性
        self.no_STM = no_STM
        self.no_camera = no_camera
        self.camera_queue = multiprocessing.Queue()
        if not no_STM:
            self.stm32_serial = Serial_handler(SERIAL_NAME, BAUDRATE)

        # 初始化各进程/线程
        if not no_camera:
            self.camera_cond = multiprocessing.Condition()
            self.camera_listener = threading.Thread(target=self.__camera_listener, name="Control_panel-Camera_listener", daemon=True)
            self.camera_process = multiprocessing.Process(target=camera_process, name="Control_panel-Camera", kwargs={"cond": self.camera_cond, "queue": self.camera_queue}, daemon=True)
            self.camera_listener.start()
            self.camera_process.start()
        if not no_STM:
            self.log_panel_thread = threading.Thread(target=self.__log_updator, name="Control_panel-Log_updator", daemon=True)
            self.log_panel_thread.start()

        # 链接各函数
        self.__connect_logic()

    def deinit(self) -> None:
        """析构函数"""
        self.running = False
        if not self.no_STM:
            self.stm32_serial.deinit()

    # 左侧指令面板
    def __steer_button(self) -> None:
        """`转向`按钮的逻辑"""
        self.stm32_serial.inst_steer(int(self.steer_angle.text()))
    def __shift_button(self) -> None:
        """`平移`按钮的逻辑"""
        self.stm32_serial.inst_shift(int(self.shift_forward.text()), int(self.shift_shift.text()))
    def __enter_grab_button(self) -> None:
        """`进入抓取状态`按钮的逻辑"""
        self.stm32_serial.inst_grab_mode()
    def __enter_place_button(self) -> None:
        """`进入放置状态`按钮的逻辑"""
        self.stm32_serial.inst_place_mode()

    # 右侧指令面板
    def __forward_button(self) -> None:
        """`↑`按钮"""
        self.stm32_serial.inst_shift(SHIFT_STEP_LENGTH, 0)
    def __backward_button(self) -> None:
        """`↓`按钮"""
        self.stm32_serial.inst_shift(-SHIFT_STEP_LENGTH, 0)
    def __left_shift_button(self) -> None:
        """`←`按钮"""
        self.stm32_serial.inst_shift(0, -SHIFT_STEP_LENGTH)
    def __right_shift_button(self) -> None:
        """`→`按钮"""
        self.stm32_serial.inst_shift(0, SHIFT_STEP_LENGTH)
    def __turn_left_button(self) -> None:
        """`左转`按钮"""
        self.stm32_serial.inst_steer(STEER_STEP_LENGTH)
    def __turn_right_button(self) -> None:
        """`右转`按钮"""
        self.stm32_serial.inst_steer(-STEER_STEP_LENGTH)

    # log_panel
    def __log_scroll_bar(self) -> None:
        """自动滚屏逻辑"""
        if self.auto_scroll.isChecked():
            scroll_bar = self.log_panel.verticalScrollBar()
            assert isinstance(scroll_bar, QScrollBar)
            scroll_bar.setValue(scroll_bar.maximum())
    def __log_once(self, raw: str, color: int) -> None:
        curr_time = time.time()
        time_str = time.strftime("%H:%M:%S", time.localtime(curr_time))
        time_str += ("%.3f" % (curr_time - int(curr_time)))[1:] # 去掉前导0

        # 保持光标位置的基础上添加日志
        textCursor = self.log_panel.textCursor()
        old_position = textCursor.position()
        textCursor.movePosition(QTextCursor.MoveOperation.End)

        charFormat = QTextCharFormat()
        charFormat.setForeground(QColor(color))
        textCursor.insertText("[%s] %s\n" % (time_str, raw), charFormat)
        textCursor.setPosition(old_position)
    def __log_updator(self) -> None:
        """`log_panel`线程, 接收并添加新信息"""
        while self.running:
            if len(self.stm32_serial.message_queue) == 0:
                with self.stm32_serial.recv_cond:
                    self.stm32_serial.recv_cond.wait()
            message = self.stm32_serial.message_queue.pop(0).decode()

            # 按不同颜色添加log
            if "FAIL" in message:
                self.__log_once(message, FAILED_COLOR)
            elif "SUCCESS" in message:
                self.__log_once(message, SUCCESS_COLOR)
            else:
                self.__log_once(message, 0)

    # 相机接收端
    def __camera_listener(self) -> None:
        while self.running:
            if self.camera_queue.empty():
                with self.camera_cond:
                    self.camera_cond.wait()
            if not self.camera_queue.empty():
                self.camera_message = self.camera_queue.get_nowait()
                self.camera_signal.emit("")
    @staticmethod
    def __array_to_pixmap(img_array: NDArray[np.uint8]) -> QPixmap:
        y, x, _ = img_array.shape
        frame = QImage(img_array.data, x, y, 3 * x, QImage.Format.Format_BGR888)
        return QPixmap.fromImage(frame)
    def __update_image_main(self) -> None:
        self.item.setPixmap(self.__array_to_pixmap(self.camera_message.rendered_picture))
        if self.camera_message.car_last_update > 0:
            self.label_position_display.setText(str(self.camera_message.car_pose[0]))
            self.label_facing_display.setText("%.1f°" % math.degrees(self.camera_message.car_pose[1]))

            time_delta = self.camera_message.timestamp - self.camera_message.car_last_update
            if time_delta < 0.2:
                self.label_positioning_method_display.setText("实时")
            else:
                self.label_positioning_method_display.setText("推断 (%.1fs)" % (time_delta))

            self.label_positioning_method_display.setStyleSheet("color: red;" if time_delta > 5 else "color: black;")

    def __connect_logic(self) -> None:
        # 左侧指令面板
        self.steer_button.clicked.connect(self.__steer_button)
        self.shift_button.clicked.connect(self.__shift_button)
        self.enter_grab_mode.clicked.connect(self.__enter_grab_button)
        self.enter_place_mode.clicked.connect(self.__enter_place_button)

        # 右侧指令面板
        self.forward_button.clicked.connect(self.__forward_button)
        self.backward_button.clicked.connect(self.__backward_button)
        self.left_shift_button.clicked.connect(self.__left_shift_button)
        self.right_shift_button.clicked.connect(self.__right_shift_button)
        self.turn_left_button.clicked.connect(self.__turn_left_button)
        self.turn_right_button.clicked.connect(self.__turn_right_button)

        if self.no_STM:
            self.command_plate.setEnabled(False)

        # log panel
        self.log_panel.verticalScrollBar().rangeChanged.connect(self.__log_scroll_bar)

        # camera
        self.camera_signal.connect(self.__update_image_main)

        self.scene = QGraphicsScene()
        self.item = QGraphicsPixmapItem()
        self.scene.setSceneRect(0, 0, CC.TRANSFORMED_WIDTH, CC.TRANSFORMED_HEIGHT)
        self.scene.addItem(self.item)
        self.graphicsView.setScene(self.scene)

        self.graphicsView.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Control_panel(no_STM=True, no_camera=True)
    win.show()

    app.exec()
    win.deinit()
