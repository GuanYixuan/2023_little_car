"""与场外相机模块相关的常数"""

import math
import numpy as np
if not __package__:
    from utils import Point
else:
    from .utils import Point

from enum import Enum
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
class Item_state(Enum):
    """物品状态枚举类"""
    VISIBLE = 1
    INVISIBLE = 2
    ON_CAR = 3
    BOUND = 4
GAUSS_BLUR_KSIZE: int = 3
"""预先进行的高斯模糊的Kernel size"""
Block_color_name = Literal["yellow", "red"]
"""所有的物块颜色"""
BLOCK_HSV_LOWERBOUND: Dict[Block_color_name, List[Tuple[int, int, int]]] = {"yellow": [(17, 96, 100)], "red": [(0, 112, 100), (170, 112, 100)]}
"""物块色彩HSV下界"""
BLOCK_HSV_UPPERBOUND: Dict[Block_color_name, List[Tuple[int, int, int]]] = {"yellow": [(33, 255, 255)], "red": [(13, 255, 255), (255, 255, 255)]}
"""物块色彩HSV上界"""
BLOCK_SIZE_THRESH: int = 25
"""物块大小阈值"""
BLOCK_LINK_MAXLENGTH: float = 0.2
"""物块就近匹配的距离阈值"""
BLOCK_OUTLIER_THRESH: float = 0.03
"""物块'在界外'的判定阈值"""
BLOCK_COMBINE_THRESH: float = 0.08
"""允许物块'粘合'与'分离'的距离阈值"""
BLOCK_DISPLAY_COLOR: Dict[Item_state, Tuple[int, int ,int]] = {Item_state.VISIBLE: (0, 255, 0), Item_state.INVISIBLE: (0, 192, 192), Item_state.ON_CAR: (192, 0, 192), Item_state.BOUND: (192, 192, 0)}
"""不同状态物块在渲染图中的标记颜色"""

# 小车定位相关常数
CAR_ERODE_KSIZE: int = 3
"""识别其它小车时, 进行的腐蚀操作的核大小"""
CAR_DILATE_KSIZE: int = 35
"""识别其它小车时, 进行的膨胀操作的核大小(用于刻画"在车附近")"""
CAR_COLOR_THRESH: List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = [((12, 6, 150), (30, 35, 250)), ((0, 0, 245), (255, 255, 255)), ((0, 0, 170), (20, 15, 230)), ((28, 0, 180), (32, 15, 210))]
"""用于识别其它小车的色彩阈值(此常数定义的是背景的阈值)"""
CAR_HOME_COLOR: Tuple[Tuple[int, int, int], Tuple[int, int, int]] = ((50, 10, 10), (140, 140, 50))
"""识别其它小车时, 目标区域的背景色"""
CAR_SIZE_THRESH: int = 2500
"""识别其它小车的大小阈值"""
CAR_BORDER_WIDTH: int = 20
"""识别其它小车时, 对场地周边进行排除的像素宽度"""
CAR_DISPLAY_COLOR: Tuple[int, int, int] = (0, 255, 0)
"""小车在渲染图中的标记颜色"""

# 目标区域相关常数
Home_names = Literal["lb", "rt"]
"""所有可能的目标区域名称"""
HOME_VERTEX: Dict[Home_names, Point] = {"lb": Point(0.2, 0.3), "rt": Point(2.8, 1.7)}
"""目标区域顶点位置, 用在Camera中"""
HOME_DISPLAY_POS: Dict[Home_names, Point] = {"lb": Point(0.05, 0.05), "rt": Point(2.75, 1.95)}
"""目标区域的显示位置, 用在Camera中"""
HOME_ENTER_POSE: Dict[Home_names, Tuple[Point, float]] = {"lb": (Point(0.3, 0.3), math.radians(180)), "rt": (Point(2.70, 1.7), 0)}
"""目标区域的进入姿态, 用于导航"""
HOME_RANGE: Dict[Home_names, Tuple[Tuple[float, float], Tuple[float, float]]] = {"lb": ((-np.inf, 0.2), (-np.inf, 0.3)), "rt": ((2.8, np.inf), (1.7, np.inf))}
"""目标区域的范围, 用于判断物块是否在区域内"""
HOME_GRIPPER_RANGE: Dict[Home_names, Tuple[Tuple[float, float], Tuple[float, float]]] = {"lb": ((-np.inf, 0.15), (-np.inf, 0.25)), "rt": ((2.85, np.inf), (1.75, np.inf))}
"""目标区域的范围, 用于判断小车的夹爪是否已到达放置区"""

HOME_NAME: Home_names = "rt"
ENEMY_HOME_NAME: Optional[Home_names] = None

HOME_DISPLAY_COLOR: Tuple[int, int, int] = (255, 80, 80)
ENEMY_HOME_DISPLAY_COLOR: Tuple[int, int, int] = (80, 80, 255)

# 系统相关常数
TARGET_UPDATE_PERIOD: float = 0.3
