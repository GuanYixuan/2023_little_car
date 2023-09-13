"""主算法相关的常数"""

import math
import numpy as np
from camera.constants import Home_names
from typing import Dict, Tuple

# 路径规划相关常数
NAV_ITEM_COLLIDE_THRESH: float = 0.0
"""路径规划时的物块碰撞阈值"""
NAV_GOTO_ANGLE_THRESH: float = math.radians(8)
"""向指定点移动时, 容许的方向误差"""
NAV_WALL_COLLIDE_THRESH: float = 0.05
"""路径规划时的边界碰撞阈值"""
NAV_COLLISION_ARM_LENGTH: float = 0.25
"""避免与墙壁碰撞时考虑的夹爪长度(从Tag中心到夹爪末端)"""
NAV_WALL_CLEAR_MOVE_DIST: float = 0.2
"""触发"远离墙壁"时, 垂直墙面移动的距离"""
NAV_MAX_FORWARD: float = 2.0
"""导航时一次前进的最长距离"""
NAV_MAX_SHIFT: float = 0.3
"""导航时一次平移的最长距离"""
NAV_HOME_ARM_LENGTH: float = 0.25
"""返回目标区域时, 为了让夹爪末端伸至区域中, 考虑的夹爪长度"""

NAV_BLOCK_APPROACH_MIN_DIST1: float = 0.6
"""接近物块时, 若初始距离大于此值则会规划一个"第一接近点", 与物块距离是`NAV_BLOCK_APPROACH_DIST1`"""
NAV_BLOCK_APPROACH_DIST1: float = 0.4
"""接近物块时, 第一接近点的位置"""
NAV_BLOCK_APPROACH_THRESH1: float = 0.15
"""接近物块时, 第一接近点的阈值"""

NAV_ITEM_NEAR_WALL_THRESH: float = 0.15
"""接近物块时, 判断物块是"靠墙物块"的阈值"""

WAIT_RESULT_TIMEOUT: Dict[str, float] = {"grab": 20.0, "place": 15.0, "steer": 8.0, "shift": 8.0}
"""不同指令接收返回值的超时时间"""
WAIT_SUCC_DELAY: float = 0.5
"""在收到SUCCESS后的额外等待时间"""


ST_AVAIL_BLOCK_LIMIT: Tuple[Tuple[float, float], Tuple[float, float]] = ((0.05, 2.95), (0.05, 1.95))
"""为防止卡墙, 允许抓取物体的范围"""
ST_TARGET_AREA_LIMIT_X: Dict[Home_names, Tuple[float, float]] = {"lb": (-np.inf, 2.35), "rt": (0.65, np.inf)}
"""对抗赛中, "第一优先抓取区域"的范围"""
