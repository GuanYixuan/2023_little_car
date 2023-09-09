# 模式定义
GRASP_PREPARE_MODE = 0 # 抓取准备
GRASP_MODE = 1 # 抓取
PLACE_PREPARE_MODE = 2 # 放置准备
PLACE_MODE = 3 # 放置
AIM_MODE = 4 # 左右对准
CHECK_MODE = 5

# 信号定义
GRAB_SUCCESS = 0
GRAB_FAILED = 1
PLACE_SUCCESS = 2
PLACE_FAILED = 3

GRAB_SUCCESS_CONTENT: str = "GRSUCC\n"
GRAB_FAILED_CONTENT: str = "GRFAIL\n"
PLACE_SUCCESS_CONTENT: str = "PLSUCC\n"
PLACE_FAILED_CONTENT: str = "PLFAIL\n"



# 传递指令
def Shift_Command(forward_mm: int, right_mm: int):
    instruction_type = 0x20
    forward_bytes = forward_mm.to_bytes(2, 'little')
    right_bytes = right_mm.to_bytes(2, 'little')

    command = bytearray([instruction_type])
    command.extend(forward_bytes)
    command.extend(right_bytes)
    while len(command) < 8:
        command.append(0x00)
    return command

def Steer_Command(angle_degrees: int):
    instruction_type = 0x10
    angle_bytes = angle_degrees.to_bytes(2, 'little')

    command = bytearray([instruction_type])
    command.extend(angle_bytes)
    while len(command) < 8:
        command.append(0x00)
    return command

def Forward_Command(signal: int): # 转发信息
    instruction_type = 0x31
    command = bytearray([instruction_type])
    if signal == GRAB_SUCCESS:
        command.extend(GRAB_SUCCESS_CONTENT.encode('utf-8'))
    elif signal == GRAB_FAILED:
        command.extend(GRAB_FAILED_CONTENT.encode('utf-8'))
    elif signal == PLACE_SUCCESS:
        command.extend(PLACE_SUCCESS_CONTENT.encode('utf-8'))
    elif signal == PLACE_FAILED:
        command.extend(PLACE_FAILED_CONTENT.encode('utf-8'))
    return command
