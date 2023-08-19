"""
导航模块, 按设计支持以下功能:

* 物品定位
* 小车定位
* 路径规划(导航)
* 获取目标区域状态

"""

import cv2
import time
import numpy as np
import pupil_apriltags
import matplotlib.pyplot as plt

import os
os.chdir(os.path.dirname(__file__))

from numpy.typing import NDArray
from typing import List, Tuple

DEBUG: bool = True

RAW_IMAGE_SHAPE: Tuple[int, int] = (1920, 1080)

TRANSFORMED_WIDTH: int = 1500
TRANSFORMED_HEIGHT: int = 1000
GAUSS_BLUR_KSIZE: int = 7

BLOCK_HSV_LOWERBOUND: Tuple[int, int, int] = (17, 128, 128)
BLOCK_HSV_UPPERBOUND: Tuple[int, int ,int] = (33, 255, 255)
BLOCK_SIZE_THRESH: int = 0

SELECT_CORNER_LINE_COLOR: Tuple[int, int, int] = (0, 196, 0)

class Navigator:
    """定位导航主类"""

    camera: cv2.VideoCapture

    image_rgb: NDArray
    """当前最新的图片, 不会自动刷新"""

    image_gray: NDArray
    """灰度化后的最新图片"""

    transformed_rgb: NDArray
    """变换后的最新图片"""

    transform_to_top:  NDArray[np.float64]
    """将原始图像变换至俯视的变换矩阵"""

    tag_detector: pupil_apriltags.Detector
    """探测Apriltag的detector"""

    def __init__(self) -> None:
        """初始化定位系统, 此过程需要手动标记场地的四个角点"""

        # 初始化相机
        self.camera = cv2.VideoCapture(0)
        assert self.camera.isOpened(), "相机未开启"
        assert RAW_IMAGE_SHAPE == (self.camera.get(cv2.CAP_PROP_FRAME_WIDTH), self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)), "分辨率校验未通过"
        self.refresh_image(True)
        self.refresh_image(True) # 初始化时第一帧是无效的, 故取两帧

        # 生成变换矩阵
        image_copy: NDArray[np.uint8] = np.copy(self.image_rgb)
        corner_list: List[Tuple[int, int]] = []
        cv2.namedWindow("select_corner")
        cv2.imshow("select_corner", image_copy)
        cv2.setMouseCallback("select_corner", self.__tag_corner_callback, param=(corner_list, image_copy))
        cv2.waitKey(-1)

        assert DEBUG
        if len(corner_list) != 4:
            corner_list = [(354, 1014), (1443, 943), (1308, 458), (557, 565)]

        assert len(corner_list) == 4
        self.transform_to_top, _unused = cv2.findHomography(np.array(corner_list), np.array([(0, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, 0), (0, 0)]))

        # 初始化tag_detector
        self.tag_detector = pupil_apriltags.Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # 尚未成型的主循环
        while True:
            self.refresh_image()
            # self.estimate_block_pose()
            self.estimate_car_pose()
            time.sleep(0.1)

    def __tag_corner_callback(self, event: int, x: int, y: int, flags: int, params: Tuple[List[Tuple[int, int]], NDArray]) -> None:
        """生成变换矩阵时的callback函数"""
        corner_list, image_copy = params
        if event == cv2.EVENT_LBUTTONUP:
            corner_list.append((x, y))

            # 绘制图示
            if len(corner_list) > 1:
                cv2.line(image_copy, (x, y), corner_list[-2], SELECT_CORNER_LINE_COLOR, 2, cv2.LINE_AA)
            if len(corner_list) == 4:
                cv2.line(image_copy, (x, y), corner_list[0], SELECT_CORNER_LINE_COLOR, 2, cv2.LINE_AA)
            cv2.drawMarker(image_copy, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 15, 2)
            cv2.imshow("select_corner", image_copy)

    def transform_points(self, input_points: "Tuple | NDArray", round: bool = False) -> NDArray:
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
        product = product[:, :2] / product[:, 2]
        if one_dim:
            product = product[0]

        if round:
            return np.round(product).astype(np.int32)
        else:
            return product

    def refresh_image(self, init: bool = False) -> None:
        """阻塞式地刷新图片"""

        # self.image_rgb = cv2.imread("test_images/desk_high.jpg") # 造点假
        # self.image_rgb = cv2.resize(self.image_rgb, (1920, 1080))

        while True:
            success, self.image_rgb = self.camera.read()
            if success:
                break
            time.sleep(0.1)

        if init:
            return

        # 进行图片变换
        self.image_gray = cv2.cvtColor(self.image_rgb, cv2.COLOR_BGR2GRAY)
        self.transformed_rgb = cv2.warpPerspective(self.image_rgb, self.transform_to_top, dsize=(TRANSFORMED_WIDTH, TRANSFORMED_HEIGHT))

    def estimate_block_pose(self) -> None:
        """识别场上的物品并计算其坐标"""

        # 模糊后进行颜色筛选
        blurred = cv2.cvtColor(cv2.GaussianBlur(self.image_rgb, (GAUSS_BLUR_KSIZE*3, GAUSS_BLUR_KSIZE*3), GAUSS_BLUR_KSIZE), cv2.COLOR_BGR2HSV)
        block_mask = cv2.inRange(blurred, BLOCK_HSV_LOWERBOUND, BLOCK_HSV_UPPERBOUND) # type: ignore

        # 分离轮廓
        raw_contours, _unused = cv2.findContours(block_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        temp_output = np.copy(self.image_rgb)
        for ind, contour in enumerate(raw_contours):
            if cv2.contourArea(contour) < BLOCK_SIZE_THRESH:
                continue

            # 取最靠屏幕下方的像素位置
            contour_img: NDArray[np.uint8] = cv2.drawContours(np.zeros(self.image_rgb.shape[:-1]), raw_contours, ind, (1,), -1)
            pixels: NDArray[np.int32] = np.where(contour_img == 1)
            bottom_ind = np.argmax(pixels[0])
            bottom_position: Tuple[int, int] = (pixels[1][bottom_ind], pixels[0][bottom_ind])

            cv2.circle(temp_output, bottom_position, 5, (0, 255, 0), -1)
            cv2.putText(temp_output, str(ind), bottom_position, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0))
            cv2.circle(self.transformed_rgb, self.transform_points(bottom_position, True), 5, (0, 255, 0), -1)

        cv2.imshow("a", self.transformed_rgb)
        cv2.waitKey(1)
        # plt.imshow(self.transformed_rgb)
        # plt.show(block=False)

    def estimate_car_pose(self) -> None:
        """根据图片更新小车位姿"""
        dets = self.tag_detector.detect(self.image_gray) # type: ignore

        for det in dets:
            cv2.circle(self.image_rgb, np.round(det.center).astype(np.int32), 10, (0, 255, 0), 2)

        cv2.imshow("w", self.image_rgb)
        cv2.waitKey(1)

if __name__ == "__main__":
    Navigator()
    
def 路径规划(小车位姿, 目标):
    小车转向目标

    前进若干距离直至接近目标
    