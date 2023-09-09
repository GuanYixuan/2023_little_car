"""主算法相关的常数"""

import math

# 路径规划相关常数
NAV_ITEM_COLLIDE_THRESH: float = 0.1
"""路径规划时的物块碰撞阈值"""
NAV_GOTO_ANGLE_THRESH: float = math.radians(5)
"""向指定点移动时, 容许的方向误差"""
NAV_WALL_COLLIDE_THRESH: float = 0.05
"""路径规划时的边界碰撞阈值"""
NAV_ARM_LENGTH: float = 0.25
"""路径规划时考虑的夹爪长度(从Tag中心到夹爪末端)"""
NAV_MAX_FORWARD: float = 1.5
"""导航时一次前进的最长距离"""
NAV_MAX_SHIFT: float = 0.3
"""导航时一次平移的最长距离"""


WAIT_SUCC_DELAY: float = 0.3
"""在收到SUCCESS后的额外等待时间"""

