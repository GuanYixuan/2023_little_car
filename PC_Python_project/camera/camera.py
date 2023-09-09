"""
场外相机模块, 按设计支持以下功能:

* 物品定位
* 小车定位
* 获取目标区域状态
"""
import os, sys
if __name__ == '__main__':
    os.chdir(os.path.dirname(__file__))
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

import cv2
import time
import math
import threading
import numpy as np
import pupil_apriltags
from multiprocessing.queues import Queue
from multiprocessing.synchronize import Condition
import matplotlib.pyplot as plt

# 从自己写的包中import
if __name__ == '__main__':
    import utils
    import constants as CC
    from constants import Item_state
    from utils import Point, Realtime_camera
else:
    from . import utils
    from . import constants as CC # camera_constants
    from .constants import Item_state
    from .utils import Point, Realtime_camera

# 类型注释系列import
from numpy.typing import NDArray
from typing import List, Tuple
from typing import Literal, Optional, Union
if __name__ == '__main__':
    from utils import Shaped_array, Shaped_NDArray
else:
    from .utils import Shaped_array, Shaped_NDArray

class Item:
    """刻画场地中的一个物品"""

    index: int
    """物品编号, 用于物品追踪"""
    state: Item_state
    """物品状态"""
    color: CC.Block_color_name
    """物品颜色"""
    coord: Point
    """物品在场地坐标系下的真实坐标"""

    bind: int
    """与这个物品'粘合'的物品index, 在`state`为`BOUND`时有效"""
    in_base: Literal[CC.Home_names, ""]
    """物品所在的基地名称, 为空则表示不在基地中"""
    on_car: int
    """物品所在的车辆(暂时不知如何刻画), 在`state`为`ON_CAR`时有效"""

    def __init__(self, index: int, state: Item_state, color: CC.Block_color_name, coord: Point) -> None:
        self.index = index
        self.state = state
        self.color = color
        self.coord = coord

        self.bind = -1
        self.in_base = ""
        for home_name in CC.HOME_RANGE:
            if self.coord.in_range(*CC.HOME_RANGE[home_name]):
                self.in_base = home_name
                break

    @property
    def pixel_pos(self) -> Point:
        """物品在俯视图中的 *像素坐标* """
        return Point(self.coord.x * (CC.TRANSFORMED_WIDTH / CC.FIELD_SIZE[0]), CC.TRANSFORMED_HEIGHT - self.coord.y * (CC.TRANSFORMED_WIDTH / CC.FIELD_SIZE[0]))

class Camera_message:
    """场外相机类回传的消息"""

    timestamp: float
    """生成该消息所用照片的接收时间戳, 采用time.monotonic()"""

    enemy_home_pos: Point
    """对方目标区域位置"""
    item_list: List[Item]
    """场地上的物品列表"""

    car_pose: Tuple[Point, float]
    """小车位姿, 表示为(真实坐标, 指向角)"""
    car_last_update: float
    """最后一次看见小车的时间, 采用time.monotonic()"""

    rendered_picture: NDArray[np.uint8]
    """叠加显示了各元素的图像"""

    def __init__(self, _timestamp: float) -> None:
        self.timestamp = _timestamp

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
    car_last_update: float
    """最后一次看见小车的时间, 采用time.monotonic()"""
    car_mask: NDArray[np.uint8]
    """被认定为小车的区域"""
    car_near_mask: NDArray[np.uint8]
    """被认定为"接近小车"的区域"""

    image_time: float
    """从`Realtime_camera`中读取到最新图片的时间, 采用time.monotonic()"""
    image_rgb: NDArray[np.uint8]
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

        * 在主线程中直接运行: 初始化后调用`main_loop`, 此时建议设置`show_render`为`True`以观察结果
        * 在子线程中运行: 先在主线程中初始化, 随后在子线程中调用`main_loop`, 此时可以利用`cond`进行同步
        * 作为独立进程运行: 直接在子进程中初始化并启动主循环, 此时可以从`queue`中接收消息并利用`cond`进行同步

        Args:
            `cond` (multiprocessing.Condition, optional): 同步变量, 若传入则会在每次主循环完成时被notify.
            `queue` (multiprocessing.Queue[Camera_message], optional): 消息队列, 若传入则会在每次主循环完成时收到回传的`Camera_message`.
            `show_render` (bool, optional): 是否弹出独立的窗口显示叠加了各元素的图像, 默认为否
        """
        np.set_printoptions(4, suppress=True)

        # 初始化各属性
        self.running = True
        self.item_list = []
        self.car_pose = (Point(-1, -1), -9)
        self.car_last_update = -1
        self.render_condition = cond
        self.message_queue = queue
        self.show_render = show_render

        # 初始化相机
        self.camera = Realtime_camera(cv2.VideoCapture('https://192.168.137.186:8080/video'), "Camera-Frame_retriever")
        assert CC.RAW_IMAGE_SHAPE == (self.camera.capture.get(cv2.CAP_PROP_FRAME_WIDTH), self.camera.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)), "分辨率校验未通过"
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
        corner_list = [(38, 626), (1102, 938), (1189, 32), (280, 123)]
        assert len(corner_list) == 4
        self.transform_to_top = cv2.getPerspectiveTransform(np.array(corner_list, dtype=np.float32),
                                            np.array([(0, CC.TRANSFORMED_HEIGHT), (CC.TRANSFORMED_WIDTH, CC.TRANSFORMED_HEIGHT), (CC.TRANSFORMED_WIDTH, 0), (0, 0)], dtype=np.float32))

        # 根据相机内参求解相机外参
        succ, rvec, tvec = cv2.solvePnP(objectPoints=np.array([(0, 0, 0), (CC.FIELD_SIZE[0], 0, 0), (*CC.FIELD_SIZE, 0), (0, CC.FIELD_SIZE[1], 0)]),
                                        imagePoints=np.array(corner_list, dtype=np.float64), cameraMatrix=CC.CAMERA_MATRIX, distCoeffs=None)
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
                cv2.line(image_copy, (x, y), corner_list[-2], CC.SELECT_CORNER_LINE_COLOR, 2, cv2.LINE_AA)
            if len(corner_list) == 4:
                cv2.line(image_copy, (x, y), corner_list[0], CC.SELECT_CORNER_LINE_COLOR, 2, cv2.LINE_AA)
            cv2.drawMarker(image_copy, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 15, 1)
            cv2.imshow("select_corner", image_copy)

    def transform_points(self, input_points: "Tuple | NDArray[np.float64]", round: bool = False) -> NDArray[np.float64]:
        """对一个或一系列坐标施加变换

        Args:
            `input_points` (Arraylike): 将要变换的坐标, 可以是一维数组或二维数组
            `round` (bool, optional): 是否对结果取整, 默认为否.

        Returns:
            `NDArray`: 经过变换的坐标, 原则上与input_points的shape一致
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

    @staticmethod
    def pixel_to_coord(inp: Point) -> Point:
        return Point(inp.x, CC.TRANSFORMED_HEIGHT - inp.y) * (CC.FIELD_SIZE[0] / CC.TRANSFORMED_WIDTH)
    @staticmethod
    def coord_to_pixel(inp: Point) -> Point:
        return Point(inp.x * (CC.TRANSFORMED_WIDTH / CC.FIELD_SIZE[0]), CC.TRANSFORMED_HEIGHT - inp.y * (CC.TRANSFORMED_WIDTH / CC.FIELD_SIZE[0]))

    def refresh_image(self, init: bool = False) -> None:
        """阻塞式地刷新图片"""

        while True:
            success, self.image_rgb = self.camera.read()
            if success:
                self.image_time = time.monotonic()
                break
            time.sleep(0.1)

        # 图片去畸变
        self.image_rgb = cv2.undistort(self.image_rgb, CC.CAMERA_MATRIX, np.array(CC.DISTORTION_COEFFICIENTS))
        self.image_gray = cv2.cvtColor(self.image_rgb, cv2.COLOR_BGR2GRAY)
        if init:
            return

        # 进行图片变换
        self.transformed_rgb = cv2.warpPerspective(self.image_rgb, self.transform_to_top, dsize=(CC.TRANSFORMED_WIDTH, CC.TRANSFORMED_HEIGHT))

    def __refresh_items(self):
        """更新物品位置, 并执行物品状态的转换"""
        new_list = self.__find_items()
        merged_list: List[Item] = []
        old_used: NDArray = np.zeros(len(self.item_list))
        new_used: NDArray = np.zeros(len(new_list))

        # 创建连边列表 Tuple[旧index, 新index, distance], 准备就近匹配
        edges: List[Tuple[int, int, float]] = []
        for old_index, old_item in enumerate(self.item_list):
            if (not old_item.state == Item_state.VISIBLE) and (not old_item.state == Item_state.INVISIBLE): # 仅有"可见"和"不可见"的物体参与连边
                continue
            for new_index, new_item in enumerate(new_list):
                if new_item.color != old_item.color: # 不同色的物体不连边
                    continue
                edges.append((old_index, new_index, old_item.coord.dist_to_point(new_list[new_index].coord)))

        # 按距离排序并建立对应关系(“就近匹配”)
        edges.sort(key=lambda tup: tup[2])
        for link in edges:
            if old_used[link[0]] or new_used[link[1]]:
                continue
            if link[2] > CC.BLOCK_LINK_MAXLENGTH: # 距离超出阈值则终止
                break
            old_used[link[0]] = new_used[link[1]] = True
            # 继承原index, 状态为可见, 继承原颜色, 采用新坐标
            merged_list.append(Item(self.item_list[link[0]].index, Item_state.VISIBLE, self.item_list[link[0]].color, new_list[link[1]].coord))

        # 对新列表中未匹配者进行检查
        for new_index, new_item in enumerate(new_list):
            if new_used[new_index]:
                continue
            # 首先检查是否为“卸货”
            # TODO

            # 其次检查是否是一次"分离"
            sep_index: int = -1
            for old_index, old_item in enumerate(self.item_list):
                if old_item.bind < 0 or old_item.color != new_item.color or old_item.coord.dist_to_point(new_item.coord) > CC.BLOCK_COMBINE_THRESH:
                    continue
                sep_index = old_index
                break
            if sep_index >= 0:
                self.item_list[sep_index].bind = -1 # 解bind
                old_used[sep_index] = True
                # 继承原index, 状态为可见, 继承原颜色, 采用新坐标
                merged_list.append(Item(self.item_list[sep_index].index, Item_state.VISIBLE, new_item.color, new_item.coord))
                continue

            # 否则新建物品, 分配新的index
            new_used[new_index] = True
            merged_list.append(Item(self.item_index_counter, Item_state.VISIBLE, new_item.color, new_item.coord))
            self.item_index_counter += 1

        # 对旧列表中未匹配者进行检查
        for old_index, old_item in enumerate(self.item_list):
            if old_used[old_index] or old_item.state == Item_state.ON_CAR:
                continue
            # 检查物体是否在车后
            pix_pos = round(old_item.pixel_pos)
            if old_item.pixel_pos.in_range((0, CC.TRANSFORMED_WIDTH), (0, CC.TRANSFORMED_HEIGHT)) and self.car_mask[pix_pos[1], pix_pos[0]]: # 暂时未用car_near_mask
                # 状态变为不可见, 此外均继承自原物体
                merged_list.append(old_item)
                merged_list[-1].state = Item_state.INVISIBLE
                continue
            # 检查是否是物体合并
            for new_index, new_item in enumerate(merged_list): # 注意此处是merged_list
                if new_item.color == old_item.color and old_item.coord.dist_to_point(new_item.coord) <= CC.BLOCK_COMBINE_THRESH:
                    # 继承原index, 状态为BOUND, 继承原颜色, 采用被绑定物体的坐标
                    merged_list.append(Item(old_item.index, Item_state.BOUND, old_item.color, new_item.coord))
                    break
            # 否则删除物体(无动作)

        # 自动继承在车上的物块
        for old_item in self.item_list:
            if old_item.state == Item_state.ON_CAR:
                merged_list.append(old_item)

        # 若被绑定物体未消失且仍在距离内, 则自动继承bind, 否则删除
        for old_index, old_item in enumerate(self.item_list):
            if old_item.state != Item_state.BOUND:
                continue
            if any(map(lambda item: (item.index == old_item.bind) and old_item.coord.dist_to_point(item.coord) <= CC.BLOCK_COMBINE_THRESH, self.item_list)):
                 merged_list.append(old_item)

        self.item_list = merged_list
    def __find_items(self) -> List[Item]:
        """识别场上的物品并计算其坐标"""
        ret: List[Item] = []

        # 模糊后进行颜色筛选
        blurred = cv2.cvtColor(cv2.GaussianBlur(self.image_rgb, (CC.GAUSS_BLUR_KSIZE, CC.GAUSS_BLUR_KSIZE), CC.GAUSS_BLUR_KSIZE/3), cv2.COLOR_BGR2HSV)
        for color in CC.BLOCK_HSV_LOWERBOUND:
            block_mask = np.zeros_like(self.image_gray)
            for lbound, ubound in zip(CC.BLOCK_HSV_LOWERBOUND[color], CC.BLOCK_HSV_UPPERBOUND[color]):
                block_mask |= cv2.inRange(blurred, lbound, ubound) # type: ignore

            # 分离轮廓
            raw_contours, _unused = cv2.findContours(block_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            for ind, contour in enumerate(raw_contours):
                if cv2.contourArea(contour) < CC.BLOCK_SIZE_THRESH:
                    continue

                # 取最靠屏幕下方的像素位置
                contour_img: NDArray[np.uint8] = cv2.drawContours(np.zeros(self.image_rgb.shape[:-1]), raw_contours, ind, (1,), -1)
                pixels: NDArray[np.int32] = np.where(contour_img == 1)
                bottom_ind = np.argmax(pixels[0])
                bottom_position: Tuple[int, int] = (pixels[1][bottom_ind], pixels[0][bottom_ind])
                transformed_position = self.transform_points(bottom_position, True)

                # 边界校验
                point = Point(*transformed_position)
                if not self.pixel_to_coord(point).in_range((-CC.BLOCK_OUTLIER_THRESH, CC.FIELD_SIZE[0]+CC.BLOCK_OUTLIER_THRESH),
                                                           (-CC.BLOCK_OUTLIER_THRESH, CC.FIELD_SIZE[1]+CC.BLOCK_OUTLIER_THRESH)):
                    continue

                # 利用car_mask排除
                if (not point.in_range((0, CC.TRANSFORMED_WIDTH), (0, CC.TRANSFORMED_HEIGHT))) or self.car_mask[point[1], point[0]] == 0:
                    ret.append(Item(-1, Item_state.VISIBLE, color, self.pixel_to_coord(Point(*transformed_position))))

        return ret

    def __estimate_car_pose(self) -> None:
        """根据图片更新小车位姿"""
        # 先尝试在原始图里面找
        dets = self.tag_detector.detect(self.image_gray, True, CC.CAMERA_PARAMS, CC.TAG_SIZE)

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
            self.car_last_update = self.image_time
            if weird_axes:
                print(weird_axes, self.car_pose[1])
            else:
                self.car_pose = (Point(*utils.pose_translation(tag_pose)[:2]), math.atan2(tag_pose[2, 0] if weird_axes else tag_pose[1, 0], tag_pose[0, 0]))

    def __estimate_other_cars(self) -> None:
        """通过排除背景而识别其它小车(以及其它同学)"""
        # 在变换后的图片上识别
        blurred = cv2.cvtColor(cv2.GaussianBlur(self.transformed_rgb, (CC.GAUSS_BLUR_KSIZE, CC.GAUSS_BLUR_KSIZE), CC.GAUSS_BLUR_KSIZE/3), cv2.COLOR_BGR2HSV)
        threshed: NDArray[np.uint8] = np.ones(blurred.shape[:2], dtype=np.uint8) # 一开始全屏都是小车
        # 随后反选背景
        for lbound, ubound in CC.CAR_COLOR_THRESH:
            threshed &= (~cv2.inRange(blurred, lbound, ubound)) # type: ignore
        # 对目标区域, 用不同的背景色反选
        target_area = np.zeros_like(threshed)
        lb = round(self.coord_to_pixel(CC.HOME_VERTEX["lb"]))
        target_area[lb[1]:, :lb[0]] = 255
        rt = round(self.coord_to_pixel(CC.HOME_VERTEX["rt"]))
        target_area[:rt[1], rt[0]:] = 255
        threshed &= (~cv2.inRange(blurred, *CC.CAR_HOME_COLOR)) | (~target_area) # type: ignore
        # 筛掉边框
        mask = np.zeros_like(threshed, dtype=np.uint8)
        mask[CC.CAR_BORDER_WIDTH:threshed.shape[0]-CC.CAR_BORDER_WIDTH, CC.CAR_BORDER_WIDTH:threshed.shape[1]-CC.CAR_BORDER_WIDTH] = 255
        threshed = mask & threshed
        cv2.imshow("threshed", threshed * 255), cv2.waitKey(1) # debug区
        # 做一些形态学变换
        threshed = cv2.erode(threshed, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CC.CAR_ERODE_KSIZE, CC.CAR_ERODE_KSIZE))) * 255
        # 提取轮廓
        self.car_mask = np.zeros_like(threshed)
        raw_contours, _unused = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for ind, contour in enumerate(raw_contours):
            area = cv2.contourArea(contour)
            if area >= CC.CAR_SIZE_THRESH: # 按大小筛选
                threshed = cv2.drawContours(threshed, raw_contours, ind, (128,), 3)
                threshed = cv2.putText(threshed, str(area), contour[0][0], cv2.FONT_HERSHEY_COMPLEX, 0.5, (128, ))
                self.car_mask = cv2.drawContours(self.car_mask, raw_contours, ind, (255,), -1)
        # 再膨胀回来
        self.car_mask = cv2.dilate(self.car_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CC.CAR_DILATE_KSIZE, CC.CAR_DILATE_KSIZE)), borderType=cv2.BORDER_REPLICATE)
    def main_loop(self) -> None:
        """场外相机主循环"""
        while self.running:
            loop_start = time.monotonic()

            self.refresh_image()
            self.__estimate_other_cars()
            self.__refresh_items()
            self.__estimate_car_pose()

            self.rendered_picture = np.copy(self.transformed_rgb)
            scale_factor: float = CC.FIELD_SIZE[0] / CC.TRANSFORMED_WIDTH
            # 绘制物品坐标
            for item in self.item_list:
                disp_color = CC.BLOCK_DISPLAY_COLOR[item.state]
                disp_nudge = Point(0, -20) if item.state == Item_state.BOUND else Point(0, 0)

                cv2.circle(self.rendered_picture, round(item.pixel_pos + disp_nudge), 2, disp_color, -1)
                out_str = "%d (%.2f, %.2f)" % (item.index, *item.coord)
                cv2.putText(self.rendered_picture, out_str, round(item.pixel_pos + disp_nudge), cv2.FONT_HERSHEY_COMPLEX, 0.5, disp_color)

            # 绘制场地
            home_render_pix = round(self.coord_to_pixel(CC.HOME_DISPLAY_POS[CC.HOME_NAME]))
            cv2.circle(self.rendered_picture, home_render_pix, 6, CC.HOME_DISPLAY_COLOR, -1)
            cv2.putText(self.rendered_picture, " (%d)" % np.count_nonzero([it.in_base == CC.HOME_NAME for it in self.item_list]),
                        home_render_pix, cv2.FONT_HERSHEY_COMPLEX, 0.5, CC.HOME_DISPLAY_COLOR)
            if isinstance(CC.ENEMY_HOME_NAME, str):
                home_render_pix = round(self.coord_to_pixel(CC.HOME_DISPLAY_POS[CC.ENEMY_HOME_NAME]))
                cv2.circle(self.rendered_picture, home_render_pix, 6, CC.ENEMY_HOME_DISPLAY_COLOR, -1)
                cv2.putText(self.rendered_picture, " (%d)" % np.count_nonzero([it.in_base == CC.ENEMY_HOME_NAME for it in self.item_list]),
                            home_render_pix, cv2.FONT_HERSHEY_COMPLEX, 0.5, CC.ENEMY_HOME_DISPLAY_COLOR)

            # 绘制小车位置与方向
            if hasattr(self, "car_pose"):
                car_pixel_pos = self.car_pose[0] / scale_factor
                car_pixel_pos.y = CC.TRANSFORMED_HEIGHT - car_pixel_pos.y
                cv2.circle(self.rendered_picture, round(car_pixel_pos), 6, CC.CAR_DISPLAY_COLOR, -1)
                cv2.arrowedLine(self.rendered_picture, round(car_pixel_pos),
                                round(car_pixel_pos + Point(math.cos(self.car_pose[1]), -math.sin(self.car_pose[1])) * 50), CC.CAR_DISPLAY_COLOR, 2, tipLength=0.2)

            # 回传消息
            if isinstance(self.message_queue, Queue):
                msg = Camera_message(self.image_time)
                msg.item_list = self.item_list
                msg.car_pose = self.car_pose
                msg.car_last_update = self.car_last_update
                msg.rendered_picture = self.rendered_picture
                self.message_queue.put(msg)

            # 叠加显示完毕, 通知其它处理程序
            if isinstance(self.render_condition, (threading.Condition, Condition)):
                with self.render_condition:
                    self.render_condition.notify_all()

            # 如有需要, 直接显示图像
            cv2.imshow("whole", self.image_rgb)
            cv2.imshow("car", self.car_mask)
            if self.show_render:
                cv2.imshow("field", self.rendered_picture)
            # 附加一个截图功能和退出功能
            key = (cv2.waitKey(1) & 0xFF)
            if key == ord('c'):
                cv2.imwrite("captures/%s.jpg" % time.strftime("%m_%d_%H_%M_%S"), self.image_rgb)
            elif key == ord('q'):
                cv2.destroyAllWindows()
                self.running = False
                del self.camera

            # 休眠一段时间
            loop_time = time.monotonic() - loop_start
            if loop_time < CC.TARGET_UPDATE_PERIOD:
                time.sleep(CC.TARGET_UPDATE_PERIOD - loop_time)
            # print("Main Loop: %.3f, items: %s" % ((time.monotonic() - loop_start), str([it.color for it in self.item_list])))

if __name__ == "__main__":
    cam = Camera(show_render=True)
    cam.main_loop()
    sys.exit(0)
