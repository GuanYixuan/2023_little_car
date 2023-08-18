"""
导航模块, 按设计支持以下功能:

* 物品定位
* 小车定位
* 路径规划(导航)
* 获取目标区域状态

"""

import cv2
import numpy as np
import pupil_apriltags
import matplotlib.pyplot as plt

import os
os.chdir(os.path.dirname(__file__))

from numpy.typing import NDArray
from typing import List, Tuple

TRANSFORMED_WIDTH: int = 1500
TRANSFORMED_HEIGHT: int = 1000
GAUSS_BLUR_KSIZE: int = 7


class Navigator:
    """定位导航主类"""

    camera: cv2.VideoCapture

    image_rgb: np.ndarray
    """当前最新的图片, 不会自动刷新"""

    transformed_rgb: np.ndarray
    """变换后的最新图片"""

    transform_to_top:  NDArray[np.float64]
    """将原始图像变换至俯视的变换矩阵"""

    def __init__(self) -> None:
        """初始化定位系统, 此过程需要手动标记场地的四个角点"""

        # 初始化相机
        # ...

        self.refresh_image(True)

        # 生成变换矩阵
        corner_list: List[Tuple[int, int]] = []
        cv2.namedWindow("select_corner")
        cv2.imshow("select_corner", self.image_rgb)
        cv2.setMouseCallback("select_corner", self.__tag_corner_callback, param=corner_list)
        cv2.waitKey(-1)

        assert len(corner_list) == 4
        self.transform_to_top, _unused = cv2.findHomography(np.array(corner_list), np.array([(0, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, TRANSFORMED_HEIGHT), (TRANSFORMED_WIDTH, 0), (0, 0)]))

    def __tag_corner_callback(self, event: int, x: int, y: int, flags: int, corner_list: List[Tuple[int, int]]) -> None:
        """生成变换矩阵时的callback函数"""
        if event == cv2.EVENT_LBUTTONUP:
            corner_list.append((x, y))

    def refresh_image(self, init: bool = False) -> None:
        """阻塞式地刷新图片"""

        self.image_rgb = cv2.imread("1692349092147.jpg") # 造点假
        self.image_rgb = cv2.resize(self.image_rgb, (1920, 1080))

        if init:
            return

        # 进行图片变换
        self.transformed_rgb = cv2.warpPerspective(self.image_rgb, self.transform_to_top, dsize=(TRANSFORMED_WIDTH, TRANSFORMED_HEIGHT))

    def estimate_block_pose(self) -> None:
        blurred = cv2.cvtColor(cv2.GaussianBlur(self.image_rgb, (GAUSS_BLUR_KSIZE*3, GAUSS_BLUR_KSIZE*3), GAUSS_BLUR_KSIZE), cv2.COLOR_BGR2HSV)

        进行颜色过滤及色块提取

        按大小筛选色块

        for 色块 in 色块s:

            获取色块中心

            利用仿射变换矩阵获取物体在场地中的坐标

    def estimate_car_pose(self) -> None:
        """根据图片更新小车位姿"""

        img: NDArray[np.uint8] = cv2.imread("apriltagrobots_overlay.jpg")

        at_detector = pupil_apriltags.Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        dets = at_detector.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))

        根据标记推断小车位姿

        for det in dets:
            cv2.circle(img, np.round(det.center).astype(np.int32), 10, (0, 255, 0), 2)

        cv2.imshow("w", img)
        cv2.imwrite("apriltagrobots_overlay_detect.jpg", img)
        cv2.waitKey(-1)


Navigator()
    
def 路径规划(小车位姿, 目标):
    小车转向目标

    前进若干距离直至接近目标
    