import math

# 测量定义
W = 320
H = 240
CY_MIN = 145
CY_MAX = 170
CX_MIN = 165
CX_MAX = 187

# 测量距离与角度
def Get_dy(obj):
    # input: obj
    # output: 与远近抓取阈值的误差(当距离过远时误差为正, 当距离过近时误差为负)
    if obj.cy() < CY_MIN:
        dy = CY_MIN - obj.cy()
    elif obj.cy() > CY_MAX:
        dy = CY_MAX - obj.cy()
    else:
        dy = 0
    return dy

def dy_to_forward(dy): # 当阈值大于0时均需要移动
    if dy <= 5 and dy > 0:
        forward_mm = 5
    elif dy <= 10 and dy > 5:
        forward_mm = 15
    elif dy <= 20 and dy > 10:
        forward_mm = 35
    elif dy > 20 and dy <= 55:
        forward_mm = 2.75 * math.sqrt(6.1 * dy - 100) + 20
    elif dy > 55:
        forward_mm = 3.65 * math.sqrt(6.25 * dy - 100) + 10
    else:
        forward_mm = 0
    return int(forward_mm)


def Get_dx(obj):
    # input: obj
    # output: 与左右抓取阈值的误差(偏左为负, 偏右为正)
    if obj.cx() < CX_MIN:
        dx = obj.cx() - CX_MIN
    elif obj.cx() > CX_MAX:
        dx = obj.cx() - CX_MAX
    else:
        dx = 0
    return dx

def dx_to_right(obj):
    if obj.cx() < CX_MIN:
        right_mm = - (obj.cx() - 200) * (obj.cx() - 200) / 300
    elif obj.cx() > CX_MAX:
        right_mm = (obj.cx() - 150) * (obj.cx() - 150) / 300
    else:
        right_mm = 0
    return int(right_mm)

def right_to_angle(right_mm):
    return int(math.atan( - right_mm / 250) * 180 / math.pi)
