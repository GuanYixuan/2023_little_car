"""
场外相机模块, 按设计支持以下功能:

* 物品定位
* 小车定位
* 获取目标区域状态

"""
import os
os.chdir(os.path.dirname(__file__))

import cv2
import time
import numpy as np
import pupil_apriltags
import matplotlib.pyplot as plt

import utils
from utils import Point

from typing import Literal, List, Tuple
from numpy.typing import NDArray
from utils import Shaped_array, Shaped_NDArray

DEBUG: bool = True

RAW_IMAGE_SHAPE: Tuple[int, int] = (1920, 1080)

TAG_SIZE: float = 0.12
CAMERA_PARAMS: Tuple[float, float, float, float] = (1.07445142e+03, 1.07705605e+03, 9.59774419e+02, 5.18931806e+02)
CAMERA_MATRIX: NDArray[np.float64] = np.array([[CAMERA_PARAMS[0], 0, CAMERA_PARAMS[2]], [0, CAMERA_PARAMS[1], CAMERA_PARAMS[3]], [0, 0, 1]])
DISTORTION_COEFFICIENTS: List[float] = [ 0.11630427, -0.23923076, -0.0081172, 0.00069115, 0.10176633]

TRANSFORMED_WIDTH: int = 700
TRANSFORMED_HEIGHT: int = 700
FIELD_SIZE: Tuple[float, float] = (0.8, 0.8)
GAUSS_BLUR_KSIZE: int = 7

BLOCK_HSV_LOWERBOUND: Tuple[int, int, int] = (17, 128, 128)
BLOCK_HSV_UPPERBOUND: Tuple[int, int ,int] = (33, 255, 255)
BLOCK_SIZE_THRESH: int = 200

SELECT_CORNER_LINE_COLOR: Tuple[int, int, int] = (0, 196, 0)

class Item:
    """刻画场地中的一个物品"""

    pos: Point
    index: int

    def __init__(self, _pos: Point, _index: int) -> None:
        self.pos = _pos
        self.index = _index

class Camera:
    """定位导航主类"""

    camera: cv2.VideoCapture

    camera_pose: Shaped_NDArray[Literal["(4,4)"], np.float64]
    """相机位姿"""

    image_rgb: NDArray
    """当前最新的图片, 已去畸变"""
    image_gray: NDArray
    """灰度化后的最新图片, 已去畸变"""
    transformed_rgb: NDArray
    """变换后的最新图片, 已去畸变"""

    transform_to_top: NDArray[np.float64]
    """将原始图像变换至俯视的变换矩阵"""

    tag_detector: pupil_apriltags.Detector
    """探测Apriltag的detector"""

    item_index_counter: int = 1
    item_list: List[Item]
    """场地上的物品列表"""

    def __init__(self) -> None:
        """初始化定位系统, 此过程需要手动标记场地的四个角点"""
        np.set_printoptions(4, suppress=True)

        # 初始化各属性
        self.item_list = []

        # 初始化相机
        self.camera = cv2.VideoCapture(0)
        assert self.camera.isOpened(), "相机未开启"
        assert RAW_IMAGE_SHAPE == (self.camera.get(cv2.CAP_PROP_FRAME_WIDTH), self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)), "分辨率校验未通过"
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
        corner_list = [(1107, 971), (1116, 30), (783, 240), (791, 765)]
        assert len(corner_list) == 4
        self.transform_to_top, _unused = cv2.findHomography(np.array(corner_list), np.array([(0, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, 0), (0, 0)]))

        # 根据相机内参求解相机外参
        succ, rvec, tvec = cv2.solvePnP(objectPoints=np.array([(0, 0, 0), (FIELD_SIZE[0], 0, 0), (*FIELD_SIZE, 0), (0, FIELD_SIZE[1], 0)]),
                                        imagePoints=np.array(corner_list, dtype=np.float64), cameraMatrix=CAMERA_MATRIX, distCoeffs=None)
        assert succ, "相机外参求解失败"
        self.camera_pose = np.linalg.inv(utils.construct_pose_cv2(rvec, tvec))

        # 初始化tag_detector
        self.tag_detector = pupil_apriltags.Detector(nthreads=4, quad_decimate=1.0)

        # 尚未成型的主循环
        while True:
            self.refresh_image()
            self.refresh_items()
            self.estimate_car_pose()

            temp_output = np.copy(self.transformed_rgb)
            for item in self.item_list:
                cv2.circle(temp_output, round(item.pos), 5, (0, 255, 0), -1)
                cv2.putText(temp_output, str(item.index), round(item.pos), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0))

            cv2.imshow("a", temp_output)
            cv2.waitKey(1)

            time.sleep(0.1)

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
            cv2.drawMarker(image_copy, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 15, 2)
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

    def refresh_items(self):
        """更新物品位置"""
        new_list = self.__find_items()
        merged_list: List[Item] = []

        # 创建连边列表 Tuple[旧index, 新index, distance]
        edges: List[Tuple[int, int, float]] = []
        for old_index, old_item in enumerate(self.item_list):
            edges.extend([(old_index, new_index, old_item.pos.dist_to_point(new_list[new_index].pos)) for new_index in range(len(new_list))])

        # 按距离排序并建立对应关系
        edges.sort(key=lambda tup: tup[2])
        old_used = np.zeros(len(self.item_list))
        new_used = np.zeros(len(new_list))
        for link in edges:
            if old_used[link[0]] or new_used[link[1]]:
                continue
            old_used[link[0]] = new_used[link[1]] = 1
            merged_list.append(Item(new_list[link[1]].pos, self.item_list[link[0]].index))

        # 让新的Item进来
        for new_index, new_item in enumerate(new_list):
            if not new_used[new_index]:
                merged_list.append(Item(new_item.pos, self.item_index_counter))
                self.item_index_counter += 1

        self.item_list = merged_list

    def __find_items(self) -> List[Item]:
        """识别场上的物品并计算其坐标"""
        ret: List[Item] = []

        # 模糊后进行颜色筛选
        blurred = cv2.cvtColor(cv2.GaussianBlur(self.image_rgb, (GAUSS_BLUR_KSIZE*3, GAUSS_BLUR_KSIZE*3), GAUSS_BLUR_KSIZE), cv2.COLOR_BGR2HSV)
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

    def estimate_car_pose(self) -> None:
        """根据图片更新小车位姿"""
        dets = self.tag_detector.detect(self.image_gray, True, CAMERA_PARAMS, TAG_SIZE) # type: ignore

        for det in dets:
            tag_pose = self.camera_pose @ utils.construct_pose_tag(det.pose_R, det.pose_t)
            print(tag_pose)
            cv2.circle(self.image_rgb, np.round(det.center).astype(np.int32), 10, (0, 255, 0), 2)

        cv2.imshow("w", self.image_rgb)
        cv2.waitKey(1)

if __name__ == "__main__":
    Camera()