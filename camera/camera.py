"""
场外相机模块, 按设计支持以下功能:

* 物品定位
* 小车定位
* 获取目标区域状态
"""
import os, sys
if __name__ == '__main__':
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

import cv2
import time
import math
import threading
import numpy as np
import pupil_apriltags
from multiprocessing.queues import Queue
from multiprocessing.synchronize import Condition

# 从自己写的包中import
if __name__ == '__main__':
    import utils
    from utils import Point, Realtime_camera
else:
    from . import utils
    from .utils import Point, Realtime_camera

# 类型注释系列import
from numpy.typing import NDArray
from typing import List, Tuple
from typing import Literal, Optional, Union
if __name__ == '__main__':
    from utils import Shaped_array, Shaped_NDArray
else:
    from .utils import Shaped_array, Shaped_NDArray

DEBUG: bool = True

TARGET_UPDATE_PERIOD: float = 0.3
RAW_IMAGE_SHAPE: Tuple[int, int] = (1280, 720)

TAG_SIZE: float = 0.12
CAMERA_PARAMS: Tuple[float, float, float, float] = (974.27198357, 976.89535927, 621.55607339, 365.47265865)
CAMERA_MATRIX: NDArray[np.float64] = np.array([[CAMERA_PARAMS[0], 0, CAMERA_PARAMS[2]], [0, CAMERA_PARAMS[1], CAMERA_PARAMS[3]], [0, 0, 1]])
DISTORTION_COEFFICIENTS: List[float] = [0.1319229, -0.28063953, 0.00082234, -0.00546549, 0.1388977]

FIELD_SIZE: Tuple[float, float] = (0.3, 0.3)
TRANSFORMED_WIDTH: int = 700
TRANSFORMED_HEIGHT: int = round(TRANSFORMED_WIDTH * FIELD_SIZE[1] / FIELD_SIZE[0]) # 保证变换后的图片与真实长度是成比例的
GAUSS_BLUR_KSIZE: int = 3

BLOCK_HSV_LOWERBOUND: Tuple[int, int, int] = (17, 96, 128)
BLOCK_HSV_UPPERBOUND: Tuple[int, int ,int] = (33, 255, 255)
BLOCK_SIZE_THRESH: int = 30
BLOCK_DISPLAY_COLOR: Tuple[int, int ,int] = (0, 255, 0)

SELECT_CORNER_LINE_COLOR: Tuple[int, int, int] = (0, 196, 0)

class Item:
    """刻画场地中的一个物品"""

    pixel_pos: Point
    """物品在俯视图中的 *像素坐标* """
    index: int

    def __init__(self, _pixel_pos: Point, _index: int) -> None:
        self.pixel_pos = _pixel_pos
        self.index = _index

    @property
    def real_coord(self) -> Point:
        """物品在场地坐标系下真实坐标"""
        return Point(self.pixel_pos.x, TRANSFORMED_HEIGHT - self.pixel_pos.y) * (FIELD_SIZE[0] / TRANSFORMED_WIDTH)

class Camera_message:
    """场外相机类回传的消息"""

    timestamp: float
    """消息生成的时间戳"""

    item_list: List[Item]
    """场地上的物品列表"""

    car_pose: Tuple[Point, float]
    """小车位姿, 表示为(真实坐标, 指向角)"""

    rendered_picture: NDArray[np.uint8]
    """叠加显示了各元素的图像"""

    def __init__(self, _item_list: List[Item], _car_pose: Tuple[Point, float], _rendered_picture: NDArray[np.uint8]) -> None:
        self.timestamp = time.monotonic()
        self.item_list = _item_list
        self.car_pose = _car_pose
        self.rendered_picture = _rendered_picture

class Camera:
    """场外相机主类"""

    running: bool

    camera: Realtime_camera
    tag_detector: pupil_apriltags.Detector
    """探测Apriltag的detector"""

    camera_pose: Shaped_NDArray[Literal["(4,4)"], np.float64]
    """相机位姿"""
    transform_to_top: NDArray[np.float64]
    """将原始图像变换至俯视的变换矩阵"""

    item_index_counter: int = 1
    item_list: List[Item]
    """场地上的物品列表"""
    car_pose: Tuple[Point, float]
    """小车位姿, 表示为(真实坐标, 指向角)"""

    image_rgb: NDArray
    """当前最新的图片, 已去畸变"""
    image_gray: NDArray
    """灰度化后的最新图片, 已去畸变"""
    transformed_rgb: NDArray
    """变换后的最新图片, 已去畸变"""
    rendered_picture: NDArray[np.uint8]
    """叠加显示了各元素的图像"""

    render_condition: Union[threading.Condition, Condition, None]
    """用于通知其它线程/进程的同步变量"""
    message_queue: Optional[Queue]
    """在相机作为独立进程运行时用于传递数据"""

    show_render: bool

    def __init__(self, *, cond: Optional[Condition] = None, queue: Optional[Queue] = None, show_render: bool = False) -> None:
        """初始化定位系统, 此过程需要手动标记场地的四个角点

        原则上讲, Camera类有以下三种使用方法:

        * 在主线程中直接运行: 初始化后调用`main_loop`
        * 在子线程中运行: 先在主线程中初始化, 随后在子线程中调用`main_loop`, 此时可以利用`cond`进行同步
        * 作为独立进程运行: 直接在子进程中初始化并启动主循环, 此时可以从`queue`中接收消息并利用`cond`进行同步

        Args:
            cond (multiprocessing.Condition, optional): 同步变量, 若传入则会在每次主循环完成时被notify.
            queue (multiprocessing.Queue[Camera_message], optional): 消息队列, 若传入则会在每次主循环完成时收到回传的`Camera_message`.
            show_render (bool, optional): 是否弹出独立的窗口显示叠加了各元素的图像, 默认为否
        """
        np.set_printoptions(4, suppress=True)

        # 初始化各属性
        self.running = True
        self.item_list = []
        self.car_pose = (Point(-1, -1), -9)
        self.render_condition = cond
        self.message_queue = queue
        self.show_render = show_render

        # 初始化相机
        self.camera = Realtime_camera(cv2.VideoCapture('https://192.168.137.186:8080/video'), "Camera-Frame_retriever")
        assert RAW_IMAGE_SHAPE == (self.camera.capture.get(cv2.CAP_PROP_FRAME_WIDTH), self.camera.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)), "分辨率校验未通过"
        self.refresh_image(True)
        self.refresh_image(True) # 初始化时第一帧是无效的, 故取两帧

        # 选取四个角点
        image_copy: NDArray[np.uint8] = np.copy(self.image_rgb)
        corner_list: List[Tuple[int, int]] = []
        cv2.namedWindow("select_corner")
        cv2.imshow("select_corner", image_copy)
        cv2.setMouseCallback("select_corner", self.__tag_corner_callback, param=(corner_list, image_copy))
        cv2.waitKey(-1)

        # 生成变换矩阵
        assert len(corner_list) == 4
        self.transform_to_top, _unused = cv2.findHomography(np.array(corner_list), np.array([(0, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, 0), (0, 0)]))

        # 根据相机内参求解相机外参
        succ, rvec, tvec = cv2.solvePnP(objectPoints=np.array([(0, 0, 0), (FIELD_SIZE[0], 0, 0), (*FIELD_SIZE, 0), (0, FIELD_SIZE[1], 0)]),
                                        imagePoints=np.array(corner_list, dtype=np.float64), cameraMatrix=CAMERA_MATRIX, distCoeffs=None)
        assert succ, "相机外参求解失败"
        self.camera_pose = np.linalg.inv(utils.construct_pose_cv2(rvec, tvec))

        # 初始化tag_detector
        self.tag_detector = pupil_apriltags.Detector(nthreads=4, quad_decimate=1.0)

    def __del__(self) -> None:
        del self.camera

    def __tag_corner_callback(self, event: int, x: int, y: int, flags: int, params: Tuple[List[Tuple[int, int]], NDArray[np.uint8]]) -> None:
        """生成变换矩阵时的callback函数"""
        corner_list, image_copy = params
        if len(corner_list) == 4: # 至多取4个点
            return

        if event == cv2.EVENT_LBUTTONUP:
            corner_list.append((x, y))

            # 绘制图示
            if len(corner_list) > 1:
                cv2.line(image_copy, (x, y), corner_list[-2], SELECT_CORNER_LINE_COLOR, 2, cv2.LINE_AA)
            if len(corner_list) == 4:
                cv2.line(image_copy, (x, y), corner_list[0], SELECT_CORNER_LINE_COLOR, 2, cv2.LINE_AA)
            cv2.drawMarker(image_copy, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 15, 1)
            cv2.imshow("select_corner", image_copy)

    def transform_points(self, input_points: "Tuple | NDArray[np.float64]", round: bool = False) -> NDArray[np.float64]:
        """对一个或一系列坐标施加变换

        Args:
            input_points (Arraylike): 将要变换的坐标, 可以是一维数组或二维数组
            round (bool, optional): 是否对结果取整, 默认为否.

        Returns:
            NDArray: 经过变换的坐标, 原则上与input_points的shape一致
        """
        input_points = np.array(input_points)
        one_dim: bool = len(input_points.shape) == 1

        if one_dim:
            input_points = np.array([*input_points, 1]).reshape((3, 1)) # 转化到 (3, 1)
        else:
            input_points = np.pad(input_points, ((0, 0), (0, 1)), 'constant', constant_values=1).T # 转化到 shape (3, N)

        product = np.dot(self.transform_to_top, input_points).T # (3, 3) * (3, N) = (3, N) -> (N, 3)
        product = product[:,:2] / product[:, 2]
        if one_dim:
            product = product[0]

        if round:
            return np.round(product).astype(np.int32)
        else:
            return product

    def refresh_image(self, init: bool = False) -> None:
        """阻塞式地刷新图片"""

        while True:
            success, self.image_rgb = self.camera.read()
            if success:
                break
            time.sleep(0.1)

        if init:
            return

        # 进行图片变换
        self.image_rgb = cv2.undistort(self.image_rgb, CAMERA_MATRIX, np.array(DISTORTION_COEFFICIENTS))
        self.image_gray = cv2.cvtColor(self.image_rgb, cv2.COLOR_BGR2GRAY)
        self.transformed_rgb = cv2.warpPerspective(self.image_rgb, self.transform_to_top, dsize=(TRANSFORMED_WIDTH, TRANSFORMED_HEIGHT))

    def __refresh_items(self):
        """更新物品位置"""
        new_list = self.__find_items()
        merged_list: List[Item] = []

        # 创建连边列表 Tuple[旧index, 新index, distance]
        edges: List[Tuple[int, int, float]] = []
        for old_index, old_item in enumerate(self.item_list):
            edges.extend([(old_index, new_index, old_item.real_coord.dist_to_point(new_list[new_index].real_coord)) for new_index in range(len(new_list))])

        # 按距离排序并建立对应关系
        edges.sort(key=lambda tup: tup[2])
        old_used = np.zeros(len(self.item_list))
        new_used = np.zeros(len(new_list))
        for link in edges:
            if old_used[link[0]] or new_used[link[1]]:
                continue
            old_used[link[0]] = new_used[link[1]] = 1
            merged_list.append(Item(new_list[link[1]].pixel_pos, self.item_list[link[0]].index))

        # 让新的Item进来
        for new_index, new_item in enumerate(new_list):
            if not new_used[new_index]:
                merged_list.append(Item(new_item.pixel_pos, self.item_index_counter))
                self.item_index_counter += 1

        self.item_list = merged_list

    def __find_items(self) -> List[Item]:
        """识别场上的物品并计算其坐标"""
        ret: List[Item] = []

        # 模糊后进行颜色筛选
        blurred = cv2.cvtColor(cv2.GaussianBlur(self.image_rgb, (GAUSS_BLUR_KSIZE, GAUSS_BLUR_KSIZE), GAUSS_BLUR_KSIZE/3), cv2.COLOR_BGR2HSV)
        block_mask = cv2.inRange(blurred, BLOCK_HSV_LOWERBOUND, BLOCK_HSV_UPPERBOUND) # type: ignore

        # 分离轮廓
        raw_contours, _unused = cv2.findContours(block_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for ind, contour in enumerate(raw_contours):
            if cv2.contourArea(contour) < BLOCK_SIZE_THRESH:
                continue

            # 取最靠屏幕下方的像素位置
            contour_img: NDArray[np.uint8] = cv2.drawContours(np.zeros(self.image_rgb.shape[:-1]), raw_contours, ind, (1,), -1)
            pixels: NDArray[np.int32] = np.where(contour_img == 1)
            bottom_ind = np.argmax(pixels[0])
            bottom_position: Tuple[int, int] = (pixels[1][bottom_ind], pixels[0][bottom_ind])
            transformed_position = self.transform_points(bottom_position, True)

            # 绘制图形(调试用)
            cv2.circle(self.transformed_rgb, transformed_position, 5, (0, 255, 0), -1)

            # 生成列表
            if Point(*transformed_position).in_range((0, TRANSFORMED_WIDTH), (0, TRANSFORMED_HEIGHT)):
                ret.append(Item(Point(*transformed_position), -1))

        return ret

    def __estimate_car_pose(self) -> None:
        """根据图片更新小车位姿"""
        dets = self.tag_detector.detect(self.image_gray, True, CAMERA_PARAMS, TAG_SIZE)

        tag_found: bool = False
        for det in dets:
            if det.decision_margin < 20: # 筛选Tag
                continue
            assert not tag_found, "找到了多于一个AprilTag"
            tag_found = True

            # 计算tag外参
            tag_pose = self.camera_pose @ utils.construct_pose_tag(det.pose_R, det.pose_t)

            # 提取平面的小车位姿
            weird_axes: bool = abs(tag_pose[2, 2]) < abs(tag_pose[1, 2]) # 假如旋转矩阵的三个轴排列方式比较奇怪
            self.car_pose = (Point(*utils.pose_translation(tag_pose)[:2]), math.atan2(tag_pose[2, 0] if weird_axes else tag_pose[1, 0], tag_pose[0, 0]))
            if weird_axes:
                print(weird_axes, self.car_pose[1])

    def main_loop(self) -> None:
        """场外相机主循环"""
        while self.running:
            loop_start = time.monotonic()

            self.refresh_image()
            self.__refresh_items()
            self.__estimate_car_pose()

            # 绘制物品坐标
            self.rendered_picture = np.copy(self.transformed_rgb)
            scale_factor: float = FIELD_SIZE[0] / TRANSFORMED_WIDTH
            for item in self.item_list:
                cv2.circle(self.rendered_picture, round(item.pixel_pos), 4, BLOCK_DISPLAY_COLOR, -1)
                out_str = "%d (%.2f, %.2f)" % (item.index, *item.real_coord)
                cv2.putText(self.rendered_picture, out_str, round(item.pixel_pos), cv2.FONT_HERSHEY_COMPLEX, 0.5, BLOCK_DISPLAY_COLOR)

            # 绘制小车位置与方向
            if hasattr(self, "car_pose"):
                car_pixel_pos = self.car_pose[0] / scale_factor
                car_pixel_pos.y = TRANSFORMED_HEIGHT - car_pixel_pos.y
                cv2.circle(self.rendered_picture, round(car_pixel_pos), 6, (0, 255, 0), -1)
                cv2.arrowedLine(self.rendered_picture, round(car_pixel_pos), round(car_pixel_pos + Point(math.cos(self.car_pose[1]), -math.sin(self.car_pose[1])) * 50), (0, 255, 0), 2, tipLength=0.2)

            # 回传消息
            if isinstance(self.message_queue, Queue):
                self.message_queue.put(Camera_message(self.item_list, self.car_pose, self.rendered_picture))

            # 叠加显示完毕, 通知其它处理程序
            if isinstance(self.render_condition, (threading.Condition, Condition)):
                with self.render_condition:
                    self.render_condition.notify_all()

            # 如有需要, 直接显示图像
            if self.show_render:
                cv2.imshow("field", self.rendered_picture)
                cv2.waitKey(1)

            # 休眠一段时间
            loop_time = time.monotonic() - loop_start
            if loop_time < TARGET_UPDATE_PERIOD:
                time.sleep(TARGET_UPDATE_PERIOD - loop_time)
            print("Main Loop: %.3f" % (time.monotonic() - loop_start))

if __name__ == "__main__":
    cam = Camera()
    cam.main_loop()
