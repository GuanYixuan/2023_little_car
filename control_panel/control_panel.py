"""小车的控制面板"""

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import time
import threading

from ui_main_window import Ui_MainWindow
from qtpy.QtWidgets import QMainWindow, QWidget, QApplication

from communication import Serial_handler

from typing import Optional

BAUDRATE: int = 9600
SERIAL_NAME: str = "COM9"

STEER_STEP_LENGTH: int = 20
SHIFT_STEP_LENGTH: int = 30

class Control_panel(QMainWindow, Ui_MainWindow):

    running: bool = True

    stm32_serial: Serial_handler

    log_panel_thread: threading.Thread

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setupUi(self)
        self.__connect_logic()

        self.stm32_serial = Serial_handler(SERIAL_NAME, BAUDRATE)
        self.log_panel_thread = threading.Thread(target=self.__log_updator, name="Panel-Log_updator", daemon=True)
        self.log_panel_thread.start()
    def deinit(self) -> None:
        """析构函数"""
        self.running = False
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

    def __log_updator(self) -> None:
        """`log_panel`线程"""
        while self.running:
            if len(self.stm32_serial.message_queue) == 0:
                with self.stm32_serial.recv_cond:
                    self.stm32_serial.recv_cond.wait()
            print(self.stm32_serial.message_queue.pop(0).decode())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Control_panel()
    win.show()

    app.exec()
    win.deinit()
