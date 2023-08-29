"""一些辅助函数或辅助类"""

import cv2
import math
import threading
import numpy as np

from typing import Iterable, Optional, Tuple, Any
from typing import TYPE_CHECKING, TypeVar, Literal, Protocol
from numpy import dtype
from numpy.typing import NDArray

if TYPE_CHECKING:
    Size = TypeVar("Size", bound=str, covariant=True)
    ScalarType = TypeVar("ScalarType", bound=np.generic, covariant=True)

    Shaped_NDArray = np.ndarray[Size, dtype[ScalarType]]
    """大小确定的np.ndarray, 用于对返回值进行类型注释"""
    class Shaped_array(np.ndarray, Protocol[Size, ScalarType]): # type: ignore
        """大小确定的数组(但不必是np.ndarray), 用于对入参进行类型注释"""
        def __getitem__(self, __i: Any) -> Any: ...
else:
    Shaped_NDArray = Tuple
    Shaped_array = Tuple

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
            raise IndexError("下标越界")
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

class Realtime_camera(threading.Thread):
    '''Always getting the most recent frame of a camera

    Modified from https://gist.github.com/crackwitz/15c3910f243a42dcd9d4a40fcdb24e40
    '''
    capture: cv2.VideoCapture

    cond: threading.Condition
    """This lets the read() method block until there's a new frame"""
    running: bool
    """This allows us to stop the thread gracefully"""
    frame: NDArray[np.uint8]
    """Keeping the newest frame around"""

    def __init__(self, capture: cv2.VideoCapture, name='FreshestFrame'):
        self.capture = capture
        assert self.capture.isOpened(), "相机未开启"

        self.cond = threading.Condition()

        self.running = False

        # passing a sequence number allows read() to NOT block
        # if the currently available one is exactly the one you ask for
        self.latestnum = 0

        super().__init__(name=name)
        self.start()

    def __del__(self):
        self.running = False
        self.join(timeout=1)
        self.capture.release()

    def start(self):
        self.running = True
        super().start()

    def run(self):
        counter = 0
        while self.running:
            # block for fresh frame
            while self.running:
                rv, img = self.capture.read()
                if rv:
                    break
            counter += 1

            # publish the frame
            with self.cond: # lock the condition for this operation
                self.frame = img # type: ignore
                self.latestnum = counter
                self.cond.notify_all()

    def read(self, wait: bool = True, seqnumber: Optional[int] = None, timeout: Optional[float] = None):
        # with no arguments (wait=True), it always blocks for a fresh frame
        # with wait=False it returns the current frame immediately (polling)
        # with a seqnumber, it blocks until that frame is available (or no wait at all)
        # with timeout argument, may return an earlier frame;
        #   may even be (0,None) if nothing received yet

        with self.cond:
            if wait:
                if seqnumber is None:
                    seqnumber = self.latestnum+1
                if seqnumber < 1:
                    seqnumber = 1

                rv = self.cond.wait_for(lambda: self.latestnum >= seqnumber, timeout=timeout)
                if not rv:
                    return (self.latestnum, self.frame)

            return (self.latestnum, self.frame)

def construct_pose_cv2(rvec: Shaped_array[Literal["(3,)"], np.float64], tvec: Shaped_array[Literal["(3,)"], np.float64]) -> Shaped_NDArray[Literal["(4,4)"], np.float64]:
    """从cv2返回的旋转向量`rvec`与平移向量`tvec`构造4x4位姿变换矩阵"""
    ret = np.zeros((4, 4), np.float64)
    ret[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
    ret[0:3, 3] = np.reshape(tvec, 3)
    ret[3, 3] = 1
    return ret
def construct_pose_tag(R: Shaped_array[Literal["(3,3)"], np.float64], t: Shaped_array[Literal["(3,)"], np.float64]) -> Shaped_NDArray[Literal["(4,4)"], np.float64]:
    """从旋转矩阵`R`与平移向量`t`构造4x4位姿变换矩阵"""
    ret = np.zeros((4, 4), np.float64)
    ret[0:3, 0:3] = R
    ret[0:3, 3] = np.reshape(t, 3)
    ret[3, 3] = 1
    return ret
def pose_translation(pose: Shaped_array[Literal["(4,4)"], np.float64]) -> Shaped_NDArray[Literal["(3,)"], np.float64]:
    """从4x4位姿变换矩阵中提取平移向量"""
    return np.reshape(pose[0:3, 3], 3)
def pose_rotation(pose: Shaped_array[Literal["(4,4)"], np.float64]) -> Shaped_NDArray[Literal["(3,3)"], np.float64]:
    """从4x4位姿变换矩阵中提取旋转矩阵"""
    return pose[0:3, 0:3]
