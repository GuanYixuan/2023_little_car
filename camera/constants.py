"""与场外相机模块相关的常数"""

import numpy as np

from typing import Tuple, List
from numpy.typing import NDArray

# 不同分辨率下的内参
CAMERA_PARAMS_1280_720: Tuple[float, float, float, float] = (974.27198357, 976.89535927, 621.55607339, 365.47265865)
CAMERA_PARAMS_1280_960: Tuple[float, float, float, float] = (991.82768552, 993.96083767, 617.66322206, 464.16969077)
DISTORTION_COEFFICIENTS_1280_720: List[float] = [0.1319229, -0.28063953, 0.00082234, -0.00546549, 0.1388977]
DISTORTION_COEFFICIENTS_1280_960: List[float] = [0.15241582, -0.23826708, -0.00905354, -0.0091075, 0.10814616]

# 相机常数
RAW_IMAGE_SHAPE: Tuple[int, int] = (1280, 960)
"""原始图像大小"""
CAMERA_PARAMS: Tuple[float, float, float, float] = CAMERA_PARAMS_1280_960
"""相机内参"""
CAMERA_MATRIX: NDArray[np.float64] = np.array([[CAMERA_PARAMS[0], 0, CAMERA_PARAMS[2]], [0, CAMERA_PARAMS[1], CAMERA_PARAMS[3]], [0, 0, 1]])
"""相机内参矩阵"""
DISTORTION_COEFFICIENTS: NDArray[np.float64] = np.array(DISTORTION_COEFFICIENTS_1280_960)
"""相机畸变参数"""

# 几何与变换常数
TAG_SIZE: float = 0.12
"""所用的AprilTag大小"""
FIELD_SIZE: Tuple[float, float] = (3.0, 2.0)
"""场地大小, 单位为米"""
TRANSFORMED_WIDTH: int = 900
"""变换后图像的宽度"""
TRANSFORMED_HEIGHT: int = round(TRANSFORMED_WIDTH * FIELD_SIZE[1] / FIELD_SIZE[0]) # 保证变换后的图片与真实长度是成比例的
"""变换后图像的高度, 目前会按照FIELD_SIZE成比例折算"""
SELECT_CORNER_LINE_COLOR: Tuple[int, int, int] = (0, 196, 0)
"""`选择角点`界面中的提示色"""

# 物品识别相关常数
GAUSS_BLUR_KSIZE: int = 3
"""预先进行的高斯模糊的Kernel size"""
BLOCK_HSV_LOWERBOUND: Tuple[int, int, int] = (17, 96, 128)
"""物块色彩HSV下界"""
BLOCK_HSV_UPPERBOUND: Tuple[int, int ,int] = (33, 255, 255)
"""物块色彩HSV上界"""
BLOCK_SIZE_THRESH: int = 30
"""物块大小阈值"""
BLOCK_DISPLAY_COLOR: Tuple[int, int ,int] = (0, 255, 0)
"""物块在渲染图中的标记颜色"""

# 小车定位相关常数
CAR_DISPLAY_COLOR: Tuple[int, int, int] = (0, 255, 0)
"""小车在渲染图中的标记颜色"""

# 系统相关常数
TARGET_UPDATE_PERIOD: float = 0.3