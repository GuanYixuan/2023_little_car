"""一些辅助函数或辅助类"""

import math
import numpy as np

from typing import Iterable, Tuple

class Point:
    """二维向量/二维点类"""
    x: float
    y: float

    def __init__(self, x0: float, y0: float) -> None:
        self.x = x0
        self.y = y0
    def __str__(self) -> str:
        return "(%.2f, %.2f)" % (self.x, self.y)
    @classmethod
    def from_tuple(cls, dot: Iterable[float]) -> "Point":
        """用tuple构造Point对象"""
        return Point(*dot)

    def __add__(self, other: "Point | float") -> "Point":
        if isinstance(other, Point):
            return Point(self.x + other.x, self.y + other.y)
        elif isinstance(other, (int, float)):
            return Point(self.x + other, self.y + other)
    def __sub__(self, other: "Point | float") -> "Point":
        if isinstance(other, Point):
            return Point(self.x - other.x, self.y - other.y)
        elif isinstance(other, (int, float)):
            return Point(self.x - other, self.y - other)
    def __mul__(self, num: float) -> "Point":
        assert isinstance(num, (int, float))
        return Point(self.x * num, self.y * num)
    def __truediv__(self, num: float) -> "Point":
        assert isinstance(num, (int, float))
        return Point(self.x / num, self.y / num)
    def __round__(self) -> Tuple[int, int]:
        """四舍五入, 返回一个Tuple"""
        return (round(self.x), round(self.y))
    def __getitem__(self, index: int) -> float:
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        else:
            raise ValueError("下标越界")
    def __len__(self) -> int:
        return 2

    def in_range(self, x_rng: Tuple[float, float] = (-np.inf, np.inf), y_rng: Tuple[float, float] = (-np.inf, np.inf)) -> bool:
        """判断此Point对象的x,y坐标是否均在给定范围内

        Args:
            x_rng (Tuple[float, float], optional): x坐标范围. Defaults to (-∞, +∞).
            y_rng (Tuple[float, float], optional): y坐标范围. Defaults to (-∞, +∞).
        """
        return x_rng[0] <= self.x and self.x < x_rng[1] and y_rng[0] <= self.y and self.y < y_rng[1]
    def to_rad(self) -> float:
        """返回该二维向量对应的幅角主值(弧度)"""
        return math.atan2(self.y, self.x)
    def self_round(self) -> "Point":
        """返回四舍五入后的Point对象"""
        return Point(round(self.x), round(self.y))
    def get_length(self) -> float:
        """返回该二维向量长度"""
        return math.sqrt(self.x*self.x + self.y*self.y)
    def dist_to_point(self, other: "Point") -> float:
        """计算该点到另一Point对象所表示的点间的欧氏距离"""
        assert isinstance(other, Point)
        return (self - other).get_length()
    def angle_to(self, other: "Point") -> float:
        """计算该从点指向另一点的向量的幅角主值"""
        assert isinstance(other, Point)
        return (other - self).to_rad()
