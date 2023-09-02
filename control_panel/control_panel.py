"""小车的控制面板"""

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import time
import threading

from ui_main_window import Ui_MainWindow
from qtpy.QtWidgets import QMainWindow, QWidget, QApplication
from qtpy.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QScrollBar
from qtpy.QtGui import QPixmap, QImage, QTextCharFormat, QColor, QTextCursor

# 从自己写的包中import
from communication import Serial_handler

# 类型注释系列import
from typing import Optional

BAUDRATE: int = 9600
SERIAL_NAME: str = "COM9"

STEER_STEP_LENGTH: int = 20
SHIFT_STEP_LENGTH: int = 30

COMMAND_COLOR: int = 0x000000
SUCCESS_COLOR: int = 0x009000
FAILED_COLOR: int = 0xa00000

class Control_panel(QMainWindow, Ui_MainWindow):
    """小车控制面板"""

    running: bool = True

    stm32_serial: Serial_handler
    """STM32串口管理对象"""

    log_panel_thread: threading.Thread

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setupUi(self)
        self.__connect_logic()

        self.stm32_serial = Serial_handler(SERIAL_NAME, BAUDRATE)
        self.log_panel_thread = threading.Thread(target=self.__log_updator, name="Control_panel-Log_updator", daemon=True)
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
            if "FAILED" in message:
                self.__log_once(message, FAILED_COLOR)
            elif "SUCCESS" in message:
                self.__log_once(message, SUCCESS_COLOR)
            else:
                self.__log_once(message, 0)

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

        # log panel
        self.log_panel.verticalScrollBar().rangeChanged.connect(self.__log_scroll_bar)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Control_panel()
    win.show()

    app.exec()
    win.deinit()
