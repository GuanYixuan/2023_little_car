"""小车的控制面板"""

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import cv2
import math
import time
import random
import threading
import numpy as np
from copy import deepcopy
import multiprocessing
from queue import Empty
from multiprocessing.queues import Queue
from multiprocessing.synchronize import Condition

from ui_main_window import Ui_MainWindow
from PyQt6.QtCore import pyqtSignal
from qtpy.QtWidgets import QMainWindow, QApplication
from qtpy.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QScrollBar
from qtpy.QtGui import QPixmap, QImage, QTextCharFormat, QColor, QTextCursor

# 从自己写的包中import
from communication import Serial_handler
from communication import STM32_INSTRUCTION_LENGTH
from communication import GRAB_SUCCESS_CONTENT, PLACE_SUCCESS_CONTENT
from camera.utils import Point
from camera.camera import Camera, Camera_message
from camera.constants import Item_state
import camera.constants as CC # camera constants
import constants as AC # algorithm constants

# 类型注释系列import
from typing import Optional, Literal
from typing import Tuple, Callable
from numpy.typing import NDArray

BAUDRATE: int = 9600
SERIAL_NAME: str = "COM7"

STEER_STEP_LENGTH: int = 30
SHIFT_STEP_LENGTH: int = 250

COMMAND_COLOR: int = 0x000000
SUCCESS_COLOR: int = 0x009000
FAILED_COLOR: int = 0xa00000

CART_STRETAGY: bool = False

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
    camera_queue: "Queue[Camera_message]"
    camera_message: Camera_message
    """最新的场外相机信息"""

    log_panel_thread: threading.Thread
    camera_process: multiprocessing.Process
    camera_listener: threading.Thread
    camera_signal = pyqtSignal(str) # 不知为何必须是class member

    alg_running: bool
    alg_start_time: float
    alg_status_update_signal = pyqtSignal(str)
    main_algorithm_thread: threading.Thread
    """主算法线程"""

    __alg_message_queue: "Queue[bytes]"

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
        self.__alg_message_queue = multiprocessing.Queue()
        if not no_STM:
            self.stm32_serial = Serial_handler(SERIAL_NAME, BAUDRATE)
            self.stm32_serial.extra_queues.append(self.__alg_message_queue)

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
        self.__log_once("发送转向指令", COMMAND_COLOR)
        self.stm32_serial.inst_steer(int(self.steer_angle.text()))
    def __shift_button(self) -> None:
        """`平移`按钮的逻辑"""
        self.__log_once("发送平移指令", COMMAND_COLOR)
        self.stm32_serial.inst_shift(int(self.shift_forward.text()), int(self.shift_shift.text()))
    def __enter_grab_button(self) -> None:
        """`进入抓取状态`按钮的逻辑"""
        self.__log_once("遥控: 进入抓取状态", COMMAND_COLOR)
        self.stm32_serial.inst_grab_mode()
    def __enter_place_button(self) -> None:
        """`进入放置状态`按钮的逻辑"""
        self.__log_once("遥控: 进入放置状态", COMMAND_COLOR)
        self.stm32_serial.inst_place_mode()

    # 右侧指令面板
    def __forward_button(self) -> None:
        """`↑`按钮"""
        self.__log_once("遥控: 前进", COMMAND_COLOR)
        self.stm32_serial.inst_shift(SHIFT_STEP_LENGTH, 0)
    def __backward_button(self) -> None:
        """`↓`按钮"""
        self.__log_once("遥控: 后退", COMMAND_COLOR)
        self.stm32_serial.inst_shift(-SHIFT_STEP_LENGTH, 0)
    def __left_shift_button(self) -> None:
        """`←`按钮"""
        self.__log_once("遥控: 左移", COMMAND_COLOR)
        self.stm32_serial.inst_shift(0, -SHIFT_STEP_LENGTH)
    def __right_shift_button(self) -> None:
        """`→`按钮"""
        self.__log_once("遥控: 右移", COMMAND_COLOR)
        self.stm32_serial.inst_shift(0, SHIFT_STEP_LENGTH)
    def __turn_left_button(self) -> None:
        """`左转`按钮"""
        self.__log_once("遥控: 左转", COMMAND_COLOR)
        self.stm32_serial.inst_steer(STEER_STEP_LENGTH)
    def __turn_right_button(self) -> None:
        """`右转`按钮"""
        self.__log_once("遥控: 右转", COMMAND_COLOR)
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
            if ("FAIL" in message.upper()) or ("ILLEAGAL" in message.upper()):
                self.__log_once(message, FAILED_COLOR)
            elif "SUCCESS" in message.upper():
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
                if self.camera_queue.empty():
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

    # 主算法部分
    def __activate_algorithm(self) -> None:
        """`启动主算法`按钮"""
        self.alg_running = True
        self.main_algorithm_thread = threading.Thread(target=self.__main_algorithm, name="Control_panel-Main_algorithm", daemon=True)
        self.main_algorithm_thread.start()

        self.activate_algorithm_button.setEnabled(False)
        self.deactivate_algorithm_button.setEnabled(True)
        self.label_alg_status_display.setText("运行中")
    def __deactivate_algorithm(self) -> None:
        """`关闭主算法`按钮"""
        self.alg_running = False
        self.main_algorithm_thread.join()
        self.activate_algorithm_button.setEnabled(True)
        self.deactivate_algorithm_button.setEnabled(False)
    def __alg_status_updater(self, msg: str) -> None:
        self.label_alg_step_display.setText(msg)

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
        if self.no_camera:
            self.label_positioning_method_display.setText("相机未启用")
        if self.no_STM or self.no_camera:
            self.label_alg_status_display.setText("不可启动")
            self.activate_algorithm_button.setEnabled(False)
            self.deactivate_algorithm_button.setEnabled(False)

        # log panel
        self.log_panel.verticalScrollBar().rangeChanged.connect(self.__log_scroll_bar) # type: ignore

        # camera
        self.camera_signal.connect(self.__update_image_main)

        scene = QGraphicsScene()
        self.item = QGraphicsPixmapItem()
        scene.setSceneRect(0, 0, CC.TRANSFORMED_WIDTH, CC.TRANSFORMED_HEIGHT)
        scene.addItem(self.item)
        self.graphicsView.setScene(scene)
        self.graphicsView.show()

        # 主算法
        self.activate_algorithm_button.clicked.connect(self.__activate_algorithm)
        self.deactivate_algorithm_button.clicked.connect(self.__deactivate_algorithm)
        self.alg_status_update_signal.connect(self.__alg_status_updater)

    # 暂且寄生在control panel下的主算法
    def __main_algorithm(self) -> None:
        grab_fail_times: int = 0
        status: str = "" # 当前状态, 仅在对付有框小车时生效
        while self.alg_running:
            # 缓存一份, 避免data race
            self.__log_once("[主算法] 刷新信息", COMMAND_COLOR)
            camera_message: Camera_message = deepcopy(self.camera_message)
            car_pos, car_angle = camera_message.car_pose

            # 占家状态
            if status == "home" and (CC.ENEMY_HOME_NAME is not None):
                if car_pos.dist_to_point(CC.HOME_OCCUPY_POS[CC.ENEMY_HOME_NAME]) < 0.1:
                    ang_d1 = Point.angle_between(car_angle, math.radians(90))
                    ang_d2 = Point.angle_between(car_angle, -math.radians(90))
                    if min(abs(ang_d1), abs(ang_d2)) <= math.radians(10):
                        time.sleep(1)
                        continue
                    elif abs(ang_d1) < abs(ang_d2): self.__nav_turn_to(math.radians(90), math.radians(10))
                    else: self.__nav_turn_to(-math.radians(90), math.radians(10))
                    self.__log_once("[主算法] 转向正确的角度", COMMAND_COLOR)
                else:
                    self.__log_once("[主算法] 向家导航", COMMAND_COLOR)
                    self.__nav_goto(CC.HOME_OCCUPY_POS[CC.ENEMY_HOME_NAME], 0.1)
                continue

            # 对付有框小车时, 检查是否能刷新状态
            if CART_STRETAGY:
                bool_list = [(item.in_base == "") and (item.state == Item_state.VISIBLE) and item.coord.in_range(*AC.ST_AVAIL_BLOCK_LIMIT) and (item.stable_frames >= 3) for item in camera_message.item_list]
                if np.count_nonzero(bool_list) == 0:
                    self.__log_once("[主算法] 进入居家状态", COMMAND_COLOR)
                    status = "home"
                    continue

            # 如果挂多了, 不妨后退变换角度
            if grab_fail_times >= 3:
                self.stm32_serial.inst_shift(-300, 50 if random.random() > 0.5 else -50)
                self.__wait_for_result("shift")
                grab_fail_times = 0
                continue

            best_index: int = -1
            best_loss: float = 1e9
            colliding: bool = False
            for ind, item in enumerate(camera_message.item_list):
                item_loss: float = 0
                if item.in_base == CC.HOME_NAME: # 不抓家里的
                    continue
                if not item.coord.in_range(*AC.ST_AVAIL_BLOCK_LIMIT): # 硬性边缘区域限制
                    continue

                # 不稳定/不可见惩罚(数量级6)
                if item.stable_frames < 10: item_loss += 1e6

                # 碰撞惩罚(数量级5)
                line_mask = np.zeros_like(camera_message.car_mask)
                cv2.line(line_mask, round(Camera.coord_to_pixel(car_pos)), round(item.pixel_pos), (255,), 2)
                collide = np.count_nonzero(camera_message.car_mask & line_mask) > 0
                if collide: item_loss += 1e5

                # 假如在"第一优先区域"中
                if item.coord.in_range(AC.ST_TARGET_AREA_LIMIT_X[CC.HOME_NAME]):
                    item_loss -= item.coord.dist_to_point(CC.HOME_CORNER_POS[CC.HOME_NAME]) # loss为负的距离(远者优先)
                else:
                    # 第二优先级惩罚(数量级4)
                    item_loss += 1e4
                    item_loss += item.coord.dist_to_point(CC.HOME_CORNER_POS[CC.HOME_NAME]) # loss为距离

                # 更新"最优的物体"
                if item_loss < best_loss:
                    best_index = ind
                    best_loss = item_loss
                    colliding = collide

            if best_index < 0:
                self.__log_once("[主算法] 未找到有效目标", COMMAND_COLOR)
                time.sleep(1)
                continue
            else:
                self.__log_once("[主算法] 向目标%s移动" % str(camera_message.item_list[best_index].coord), COMMAND_COLOR)

            # 导航过去前, 先比较抽象地远离边缘
            if not car_pos.in_range((AC.NAV_COLLISION_ARM_LENGTH, CC.FIELD_SIZE[0] - AC.NAV_COLLISION_ARM_LENGTH), (AC.NAV_COLLISION_ARM_LENGTH, CC.FIELD_SIZE[1] - AC.NAV_COLLISION_ARM_LENGTH)):
                if car_pos.x < AC.NAV_COLLISION_ARM_LENGTH:
                    self.stm32_serial.inst_shift(*round(Point(math.cos(car_angle), math.sin(car_angle)) * AC.NAV_WALL_CLEAR_MOVE_DIST * 1000))
                elif car_pos.y < AC.NAV_COLLISION_ARM_LENGTH:
                    self.stm32_serial.inst_shift(*round(Point(math.sin(car_angle), -math.cos(car_angle)) * AC.NAV_WALL_CLEAR_MOVE_DIST * 1000))
                elif car_pos.x > CC.FIELD_SIZE[0] - AC.NAV_COLLISION_ARM_LENGTH:
                    self.stm32_serial.inst_shift(*round(Point(math.cos(car_angle), math.sin(car_angle)) * -AC.NAV_WALL_CLEAR_MOVE_DIST * 1000))
                elif car_pos.y > CC.FIELD_SIZE[1] - AC.NAV_COLLISION_ARM_LENGTH:
                    self.stm32_serial.inst_shift(*round(Point(-math.sin(car_angle), math.cos(car_angle)) * AC.NAV_WALL_CLEAR_MOVE_DIST * 1000))
                self.__wait_for_result("shift")
                continue # 要求重新刷新

            # 导航过去
            target_item = camera_message.item_list[best_index]
            target_coord = target_item.coord
            self.alg_status_update_signal.emit("接近物块")
            vec_CI = target_item.coord - car_pos
            if vec_CI.get_length() > AC.NAV_BLOCK_APPROACH_MIN_DIST1:
                near_wall_adjuster = Point(0, 0)
                if target_coord.x < AC.NAV_ITEM_NEAR_WALL_THRESH: near_wall_adjuster += Point(1, 0)
                if target_coord.y < AC.NAV_ITEM_NEAR_WALL_THRESH: near_wall_adjuster += Point(0, 1)
                if target_coord.x > CC.FIELD_SIZE[0] - AC.NAV_ITEM_NEAR_WALL_THRESH: near_wall_adjuster += Point(-1, 0)
                if target_coord.y > CC.FIELD_SIZE[1] - AC.NAV_ITEM_NEAR_WALL_THRESH: near_wall_adjuster += Point(0, -1)

                if near_wall_adjuster.get_length() >= 0.5: # 如果物块接近墙
                    self.__nav_goto(target_coord + near_wall_adjuster * AC.NAV_BLOCK_APPROACH_DIST1 / near_wall_adjuster.get_length(), AC.NAV_BLOCK_APPROACH_THRESH1,
                                    stop_crit=lambda msg: self.__to_block_near_crit(msg, target_coord), max_single_action_dist=0.999 if colliding else 100)
                else: # 如果物块不接近墙的话
                    self.__nav_goto(car_pos + vec_CI * (1 - AC.NAV_BLOCK_APPROACH_DIST1 / vec_CI.get_length()), AC.NAV_BLOCK_APPROACH_THRESH1,
                                    stop_crit=lambda msg: self.__to_block_near_crit(msg, target_coord), max_single_action_dist=0.999 if colliding else 100)
                continue # 要求重新刷新
            if vec_CI.get_length() > 0.33:
                self.__nav_goto(car_pos + vec_CI * (1 - 0.28 / vec_CI.get_length()), 0.05)
            car_pos = deepcopy(self.camera_message.car_pose[0]) # 刷新?
            self.__nav_turn_to(car_pos.angle_to(target_item.coord), math.radians(5))

            self.alg_status_update_signal.emit("拾取物块")
            self.__log_once("[主算法] 进入抓取模式", COMMAND_COLOR)
            self.stm32_serial.inst_grab_mode()
            if not self.__wait_for_result("grab"):
                grab_fail_times += 1
                continue

            # 导航回家
            self.alg_status_update_signal.emit("返回目标区")
            self.__log_once("[主算法] 返回目标区", COMMAND_COLOR)
            while True:
                self.__nav_goto(CC.HOME_ENTER_POSE[CC.HOME_NAME][0], 0.05, stop_crit=self.__to_home_near_crit)
                if self.__to_home_near_crit(self.camera_message): break
                self.__nav_turn_to(CC.HOME_ENTER_POSE[CC.HOME_NAME][1], math.radians(5), stop_crit=self.__to_home_stop_crit)
                if self.__to_home_stop_crit(self.camera_message): break
                self.stm32_serial.inst_shift(0, -150)
                if self.__to_home_stop_crit(self.camera_message): break
                self.stm32_serial.inst_shift(50, 0)
                self.__wait_for_result("shift")
                break
            if not self.__to_home_stop_crit(self.camera_message): # 已到达目标区域附近, 只需要一个旋转
                assert self.__to_home_near_crit(self.camera_message)
                car_pos, car_angle = self.camera_message.car_pose

                min_turn: int = 1000
                for deg_add in range(-180, 185, 5):
                    if (car_pos + Point(math.cos(car_angle + math.radians(deg_add)), math.sin(car_angle + math.radians(deg_add))) * AC.NAV_HOME_ARM_LENGTH).in_range(*CC.HOME_GRIPPER_RANGE[CC.HOME_NAME]):
                        if abs(deg_add) < abs(min_turn):
                            min_turn = deg_add
                if abs(min_turn) <= 180:
                    self.stm32_serial.inst_steer(min_turn)
                    self.__wait_for_result("steer")
                else:
                    self.__nav_turn_to(car_pos.angle_to(CC.HOME_CORNER_POS[CC.HOME_NAME]), math.radians(10))

            self.alg_status_update_signal.emit("放置物块")
            self.__log_once("[主算法] 进入放置模式", COMMAND_COLOR)
            self.stm32_serial.inst_place_mode()
            self.__wait_for_result("place") # 放置失败, 不知如何处理

            self.__log_once("[主算法] 放置成功, 离开", COMMAND_COLOR)
            self.stm32_serial.inst_shift(-200, 150)
            self.__wait_for_result("shift")
    def __wait_for_result(self, command_type: Literal["grab", "place", "shift", "steer"]) -> bool:
        """阻塞式地等待指令的结果, 不同指令的超时时间由`AC.WAIT_RESULT_TIMEOUT`常量指定"""
        ret: bool = False
        timeout: float = AC.WAIT_RESULT_TIMEOUT[command_type]
        start_time: float = time.monotonic()
        while time.monotonic() - start_time < timeout:
            try:
                message = self.__alg_message_queue.get(timeout=1).decode()
            except Empty:
                continue
            if (command_type == "shift" or command_type == "steer") and message.startswith("SUCCESS: %s" % command_type):
                ret = True
                break
            if len(message) != STM32_INSTRUCTION_LENGTH - 2:
                continue
            if message.startswith("GR") and command_type == "grab":
                ret = (message == GRAB_SUCCESS_CONTENT)
                break
            if message.startswith("PL") and command_type == "place":
                ret = (message == PLACE_SUCCESS_CONTENT)
                break
        if time.monotonic() - start_time >= timeout:
            self.__log_once("[主算法] 等待指令结果超时", FAILED_COLOR)
            return ret
        time.sleep(AC.WAIT_SUCC_DELAY)
        wait_time = time.monotonic()
        while self.camera_message.car_last_update < wait_time:
            with self.camera_cond:
                self.camera_cond.wait()
        return ret
    def __nav_goto(self, target_pos: Point, thresh: float, *, shift_preferred: bool = False, max_adjust: int = 10000,
                   stop_crit: Optional[Callable[[Camera_message], bool]] = None, max_single_action_dist: float = 100) -> int:
        """让小车行驶至指定位置

        Args:
            `target_pos` (Point): 目标位置
            `thresh` (float): 允许的位置偏差, 单位为米
            `shift_preferred` (bool, optional): 是否采用左右平移的方式进行移动, 默认为否
            `stop_crit` (Callable[[Camera_message], bool], optional): 提前停止的条件, 默认不提前停止
            `max_single_action_dist` (float, optional): 单次移动的最大距离, 注意此参数不会覆盖`AC.NAV_MAX_FORWARD`的限制作用, 默认无限制
        """
        adjust_count: int = 0
        while adjust_count < max_adjust:
            camera_message: Camera_message = deepcopy(self.camera_message)
            car_pos, car_angle = camera_message.car_pose
            target_distance: float = car_pos.dist_to_point(target_pos)
            target_angle: float = car_pos.angle_to(target_pos)
            # 进行优先检查
            if (stop_crit is not None) and stop_crit(camera_message):
                return adjust_count
            # 在误差范围内则返回
            if target_distance < thresh:
                return adjust_count
            # 否则旋转后进行移动
            if not shift_preferred:
                if self.__nav_turn_to(target_angle, AC.NAV_GOTO_ANGLE_THRESH, max_adjust=10, stop_crit=stop_crit):
                    continue
                # 计算前进距离
                limited_dist = min(target_distance, AC.NAV_MAX_FORWARD, max_single_action_dist)
                limited_dist = min(limited_dist, max(self.__check_wall_collision_dist(car_pos, target_angle) - AC.NAV_COLLISION_ARM_LENGTH - AC.NAV_WALL_COLLIDE_THRESH, 0))
                limited_dist = round(limited_dist * 1000)
                # if limited_dist >= 1000: input("确认指令: 前进%d" % limited_dist)
                self.stm32_serial.inst_shift(limited_dist, 0)
            else:
                rshift_dist = Point.angle_between(car_angle, target_angle - math.radians(90))
                lshift_dist = Point.angle_between(car_angle, target_angle + math.radians(90))
                target_angle = target_angle - math.radians(90) if rshift_dist < lshift_dist else target_angle + math.radians(90)
                shift_sign = 1 if rshift_dist < lshift_dist else -1
                if self.__nav_turn_to(target_angle, AC.NAV_GOTO_ANGLE_THRESH, max_adjust=10, stop_crit=stop_crit):
                    continue
                # 计算平移距离
                limited_dist = min(target_distance, AC.NAV_MAX_SHIFT, max_single_action_dist)
                limited_dist = min(limited_dist, max(self.__check_wall_collision_dist(car_pos, target_angle) - AC.NAV_WALL_COLLIDE_THRESH, 0)) # 不再加上夹爪长度
                limited_dist = round(limited_dist * 1000 * shift_sign)
                input("确认指令: 右移%d" % limited_dist)
                self.stm32_serial.inst_shift(0, limited_dist)
            self.__wait_for_result("shift")
            adjust_count += 1
        return adjust_count
    def __nav_turn_to(self, target_angle: float, thresh: float, *, max_adjust: int = 10000, stop_crit: Optional[Callable[[Camera_message], bool]] = None) -> int:
        """将小车转至指定方向

        Args:
            `target_angle` (float): 目标方向, 单位为弧度
            `thresh` (float): 允许的角度偏差, 单位为弧度
            `max_adjust` (int, optional): 最大调整次数, 默认无限制
            `stop_crit` (Callable[[Camera_message], bool], optional): 提前停止的条件, 默认不提前停止
        """
        adjust_count: int = 0
        while adjust_count < max_adjust:
            camera_message: Camera_message = deepcopy(self.camera_message)
            target_angle_diff: float = Point.angle_between(camera_message.car_pose[1], target_angle)
            # 进行优先检查
            if (stop_crit is not None) and stop_crit(camera_message):
                return adjust_count
            # 在误差范围内则返回
            if abs(target_angle_diff) <= thresh:
                return adjust_count
            # 否则发出旋转指令
            turn_angle: int = round(math.degrees(target_angle_diff))
            self.stm32_serial.inst_steer(turn_angle)
            self.__wait_for_result("steer")

            adjust_count += 1
        return adjust_count
    @staticmethod
    def __check_wall_collision_dist(curr_pos: Point, angle: float) -> float:
        ret: float = 1e9
        right_bound: float = (CC.FIELD_SIZE[0] - curr_pos.x) / math.cos(angle)
        left_bound: float = (-curr_pos.x) / math.cos(angle)
        upper_bound: float = (CC.FIELD_SIZE[1] - curr_pos.y) / math.sin(angle)
        lower_bound: float = (-curr_pos.y) / math.sin(angle)
        for item in (right_bound, left_bound, upper_bound, lower_bound):
            ret = ret if item < 0 else min(ret, item)
        return ret
    @staticmethod
    def __to_block_near_crit(msg: Camera_message, target_pos: Point) -> bool:
        return msg.car_pose[0].dist_to_point(target_pos) <= AC.NAV_BLOCK_APPROACH_MIN_DIST1
    @staticmethod
    def __to_home_stop_crit(msg: Camera_message) -> bool:
        car_pos, car_angle = msg.car_pose
        return (car_pos + Point(math.cos(car_angle), math.sin(car_angle)) * AC.NAV_HOME_ARM_LENGTH).in_range(*CC.HOME_GRIPPER_RANGE[CC.HOME_NAME])
    @staticmethod
    def __to_home_near_crit(msg: Camera_message) -> bool:
        return msg.car_pose[0].in_range(*CC.HOME_NEAR_RANGE[CC.HOME_NAME])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Control_panel(no_STM=False, no_camera=False)
    win.show()

    app.exec()
    win.deinit()
