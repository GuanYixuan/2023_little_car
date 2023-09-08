"""与场外相机模块相关的常数"""

import numpy as np
if not __package__:
    from utils import Point
else:
    from .utils import Point

from typing import Tuple, List, Dict, Literal
from typing import Optional
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
BLOCK_HSV_LOWERBOUND: Dict[str, List[Tuple[int, int, int]]] = {"yellow": [(17, 96, 128)], "red": [(0, 96, 128), (175, 96, 128)]}
"""物块色彩HSV下界"""
BLOCK_HSV_UPPERBOUND: Dict[str, List[Tuple[int, int, int]]] = {"yellow": [(33, 255, 255)], "red": [(13, 255, 255), (180, 255, 255)]}
"""物块色彩HSV上界"""
BLOCK_SIZE_THRESH: int = 25
"""物块大小阈值"""
BLOCK_OUTLIER_THRESH: float = 0.08
"""物块'在界外'的判定阈值"""
BLOCK_COMBINE_THRESH: float = 0.08
"""允许物块'粘合'与'分离'的距离阈值"""
BLOCK_DISPLAY_COLOR: Tuple[int, int ,int] = (0, 255, 0)
"""一般物块在渲染图中的标记颜色"""
BLOCK_COMBINED_COLOR: Tuple[int, int ,int] = (192, 192, 0)
"""'合并'的物块在渲染图中的标记颜色"""

# 小车定位相关常数
CAR_ERODE_KSIZE: int = 5
"""识别其它小车时, 进行的腐蚀操作的核大小"""
CAR_DILATE_KSIZE: int = 30
"""识别其它小车时, 进行的膨胀操作的核大小(用于表述"在车附近")"""
CAR_COLOR_THRESH: List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = [((12, 6, 150), (30, 35, 250)), ((0, 0, 245), (255, 255, 255)), ((0, 0, 170), (20, 15, 210))]
"""用于识别其它小车的色彩阈值(此常数定义的是背景的阈值)"""
CAR_SIZE_THRESH: int = 3000
"""识别其它小车的大小阈值"""
CAR_BORDER_WIDTH: int = 25
"""识别其它小车时, 对场地周边进行排除的像素宽度"""
CAR_DISPLAY_COLOR: Tuple[int, int, int] = (0, 255, 0)
"""小车在渲染图中的标记颜色"""

# 目标区域相关常数
Home_names = Literal["lb", "rt"]
AVAILABLE_HOME_POSE: Dict[Home_names, Point] = {"lb": Point(0.1, 0.2), "rt": Point(2.9, 1.8)}
HOME_RANGE: Dict[Home_names, Tuple[Tuple[float, float], Tuple[float, float]]] = {"lb": ((-np.inf, 0.2), (-np.inf, 0.3)), "rt": ((2.8, np.inf), (1.7, np.inf))}

HOME_NAME: Home_names = "lb"
HOME_POS: Point = AVAILABLE_HOME_POSE[HOME_NAME]

ENEMY_HOME_NAME: Optional[Home_names] = "rt"
ENEMY_HOME_POS: Optional[Point] = None if not ENEMY_HOME_NAME else AVAILABLE_HOME_POSE[ENEMY_HOME_NAME]
HOME_DISPLAY_COLOR: Tuple[int, int, int] = (255, 80, 80)
ENEMY_HOME_DISPLAY_COLOR: Tuple[int, int, int] = (80, 80, 255)

# 系统相关常数
TARGET_UPDATE_PERIOD: float = 0.3
