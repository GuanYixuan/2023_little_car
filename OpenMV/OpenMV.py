# Untitled - By: Lenovo - 周一 8月 28 2023

import sensor, image, time
from pyb import UART
from pyb import Servo
import math
import message
import measure

# 颜色阈值
green_threshold  = (26, 62, -42, -14, 12, 33)
orange_threshold = (20, 90, 15, 60, 20, 100)
yellow_threshold = (39, 85, -27, 6, 32, 90) #黄色物块
all_threshold = [(26, 62, -42, -14, 12, 33), # green
                 (20, 90, 15, 60, 20, 100)]  # orange
black_threshold = (0, 38, -20, 3, 1, 23) # 家
home_threshold = [
    (26, 62, -42, -14, 12, 33), # green_threshold
    (20, 90, 15, 60, 20, 100), # orange_threshold
    (0, 3, -7, -2, 1, 10) # black_threshold
]

# 串口定义
uart = UART(3, 9600)

# 舵机定义
open_servo = Servo(1)
up_down_servo = Servo(2)
GRASP_ANGLE = -55
PLACE_ANGLE = 0
ARMDOWN_ANGLE = -60
ARMUP_ANGLE = -20
ARMMID_ANGLE = -30

# 测量定义
W = 320
H = 240
CY_MIN = 100
CY_MAX = 130
CX_MIN = 165
CX_MAX = 187



# 模式定义
GRASP_PREPARE_MODE = 0 # 抓取准备
GRASP_MODE = 1 # 抓取
PLACE_PREPARE_MODE = 2 # 放置准备
PLACE_MODE = 3 # 放置
AIM_MODE = 4 # 左右对准
CHECK_MODE = 5
TRANSPORT_MODE = 6

# 信号定义
GRAB_SUCCESS = 0
GRAB_FAILED = 1
PLACE_SUCCESS = 2
PLACE_FAILED = 3

GRAB_SUCCESS_CONTENT: str = "GRSUCC\n"
GRAB_FAILED_CONTENT: str = "GRFAIL\n"
PLACE_SUCCESS_CONTENT: str = "PLSUCC\n"
PLACE_FAILED_CONTENT: str = "PLFAIL\n"


# 待测常数
ANGLE_MAX: int = 20 # 待测!
DISTANCE_MAX: int = 150
DISTANCE_MIN: int = 100


# 变量初始化
angle_aim_times = 0
distance_aim_times = 0


def Sensor_Init():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA) #320*240
    sensor.skip_frames(time = 2000)
    sensor.set_auto_gain(False) #关闭自动增益
    sensor.set_auto_whitebal(False) #关闭白平衡

# 判断是否为绿色
def Green_Judge(obj):
    green_roi = obj.rect()
    if img.find_blobs(green_threshold, roi = green_roi, pixels_threshold = 100, merge = True):
        return True
    else:
        return False

# 夹持控制
def Servo_Init():
    open_servo.angle(PLACE_ANGLE)
    up_down_servo.angle(ARMDOWN_ANGLE)

def Servo_Control(mode):
    if mode == 0: # 找物块
        open_servo.angle(PLACE_ANGLE)
        time.sleep_ms(200)
        up_down_servo.angle(ARMDOWN_ANGLE)
    elif mode == 1: # 抓物块
        open_servo.angle(GRASP_ANGLE)
        time.sleep_ms(200)
        up_down_servo.angle(ARMUP_ANGLE, 1000)
    elif mode == 2: # 检测是否抓到物块
        open_servo.angle(GRASP_ANGLE)
        time.sleep_ms(500)
        up_down_servo.angle(ARMUP_ANGLE, 1000)
    elif mode == 3: # 夹持物块移动
        open_servo.angle(GRASP_ANGLE)
        up_down_servo.angle(ARMMID_ANGLE, 1000)
    elif mode == 4: # 放置物块
        up_down_servo.angle(ARMDOWN_ANGLE, 1000)
        time.sleep_ms(200)
        open_servo.angle(PLACE_ANGLE, 1000)

Sensor_Init()
Servo_Init()
clock = time.clock() # 追踪频率

# 初始化
Servo_Control(0)
shift_times = 0

left_right_aim_times = 0
forward_aim_times = 0
left_right_change_times = 0
forward_change_times = 0

last_change_time = 0
up_roi = (80, 0, 160, 50)
home_center_roi = (118, 140, 128, 40)
home_left_roi = (40, 0, 100, 200)
home_right_roi = (240, 0, 80, 200)
home_front_roi = (140, 0, 100, 70)

mode = PLACE_PREPARE_MODE

uart_time = time.time()
# 主循环
while(True):
    clock.tick()
    img = sensor.snapshot()

    # 读取指令
    if uart.any():
        command_receive = uart.readline().decode().strip()
        if command_receive == "GRASP":
            left_right_change_times = 0
            forward_change_times = 0
            mode: int = GRASP_PREPARE_MODE
        elif command_receive == "PLACE":
            left_right_change_times = 0
            forward_change_times = 0
            mode: int = PLACE_PREPARE_MODE


    # 抓取 TODO: 3.角度移动 思考: 物块在视野内平移, 物块不在视野内旋转?
    if mode == GRASP_PREPARE_MODE:
        Servo_Control(0)
        img = img.binary([orange_threshold], invert=True, zero=True)  # 1 二值化
        blobs = img.find_blobs([orange_threshold], merge=False) # 2 找色块
        if blobs:
            # 找出像素面积最大的物块
            max_b = blobs[0]
            for b in blobs:
                if b.pixels() >= max_b.pixels():
                    max_b = b
            img.draw_rectangle(max_b[0:4])
            img.draw_cross(max_b[5], max_b[6])
            print("AIM AIM AIM!")
            mode = AIM_MODE

    if mode == AIM_MODE:
        Servo_Control(0)
        img = img.binary([orange_threshold], invert=True, zero=True)  # 1 二值化
        blobs = img.find_blobs([orange_threshold], merge=False) # 2 找色块
        if blobs:
            # 找出像素面积最大的物块
            max_b = blobs[0]
            for b in blobs:
                if b.pixels() >= max_b.pixels():
                    max_b = b
            img.draw_rectangle(max_b[0:4])
            img.draw_cross(max_b[5], max_b[6])

            right_mm = measure.dx_to_right(max_b)
            angle_degrees = measure.right_to_angle(right_mm)
            if angle_degrees <= 1 and angle_degrees >= -1:   # 角度在-1与1之间, 不需调整, 视为角度对准
                left_right_aim_times += 1                    # 角度瞄准次数+1
            else:
                print("angle", angle_degrees)
                left_right_aim_times = 0                     # 角度需要调整, 角度瞄准次数归零
                left_right_change_times += 1                 # 左右调整次数+1, 自进入瞄准状态后的第一次调整不需要等成功信息, 否则需要等成功信息
                if left_right_change_times == 1:
                    print("uart write angle 1")
                    uart.write(message.Steer_Command(angle_degrees))
                    uart_time = time.time()
                else:
                    if uart.any():
                        command_receive = uart.readline().decode().strip()
                        if command_receive == "EXECEND":
                            uart.write(message.Steer_Command(angle_degrees))
                            uart_time = time.time()
                            print("uart write angle 2")
                        else:
                            continue
                    else:
                        delta_time = time.time() - uart_time
                        if delta_time > 0.5:
                            uart.write(message.Steer_Command(angle_degrees))
                            uart_time = time.time()
                        continue

            if left_right_aim_times >= 5:                    # 角度瞄准五次后进行前后调整
                print("left-right ok!")
                dy = measure.Get_dy(max_b)
                forward_mm = measure.dy_to_forward(dy)

                if forward_mm == 0:                          # 前后不需调整, 前后瞄准次数+1
                    forward_aim_times += 1
                else:
                    print("forward", forward_mm)
                    forward_aim_times = 0                    # 前后需要调整, 前后瞄准次数归零
                    forward_change_times += 1
                    if forward_change_times == 1:
                        print("uart write forward 1")
                        uart.write(message.Shift_Command(forward_mm, 0))
                        uart_time = time.time()
                    else:
                        if uart.any():
                            command_receive = uart.readline().decode().strip()
                            if command_receive == "EXECEND":
                                print("uart write forward 2")
                                uart.write(message.Shift_Command(forward_mm, 0))
                                uart_time = time.time()
                            else:
                                continue
                        else:
                            delta_time = time.time() - uart_time
                            if delta_time > 0.5:
                                uart.write(message.Shift_Command(forward_mm, 0))
                                uart_time = time.time()
                            else:
                                continue


                if forward_aim_times >= 5:
                    print("抓抓抓!")
                    mode = GRASP_MODE


    if mode == GRASP_MODE:
        Servo_Control(1)
        time.sleep_ms(500)
        Servo_Control(2)
        print("CHECK CHECK CHECK!")
        mode = CHECK_MODE

    if mode == CHECK_MODE:
        time.sleep(1)
        img = sensor.snapshot()
        blobs = img.find_blobs([orange_threshold], roi= up_roi, pixels_threshold = 100, merge=False)
        if blobs:
            for b in blobs:
                img.draw_rectangle(b[0:4])
                img.draw_cross(b[5], b[6])

            command = message.Forward_Command(GRAB_SUCCESS)
            uart.write(command)
            Servo_Control(3)
            print("success")
            mode = TRANSPORT_MODE
        else:
            command = message.Forward_Command(GRAB_FAILED)
            uart.write(command)
            print("failed")
            mode = GRASP_PREPARE_MODE
        # 检查爪子里是否有物块, 如果有则发送抓取成功的转发指令
        # if obj in grasp
        # uart.write(Forward_Command(GRAB_SUCCESS))

        # else
        # uart.write(Forward_Command(GRAB_FAILED))

    if mode == TRANSPORT_MODE:
        Servo_Control(3)

    if mode == PLACE_PREPARE_MODE:
        #time.sleep(3)
        #print("place prepare")
        Servo_Control(3)
        img = sensor.snapshot()
        black_blobs_left = img.find_blobs([black_threshold], roi = home_left_roi, pixels_threshold = 100, merge = True)
        black_blobs_right = img.find_blobs([black_threshold], roi = home_right_roi, pixels_threshold = 100, merge = True)
        black_blobs_center = img.find_blobs([black_threshold], roi = home_center_roi, pixels_threshold = 1000, merge = True)
        black_blobs_front = img.find_blobs([black_threshold], roi = home_front_roi, pixels_threshold = 1000, merge = True)

        if black_blobs_left:
            for b in black_blobs_left:
                left_max_b = black_blobs_left[0]
                if b.pixels() > left_max_b.pixels():
                    left_max_b = b
                img.draw_rectangle(left_max_b[0:4], color = (255, 0, 0))
                img.draw_cross(left_max_b[5], left_max_b[6])
        if black_blobs_right:
            for b in black_blobs_right:
                right_max_b = black_blobs_right[0]
                if b.pixels() > right_max_b.pixels():
                    right_max_b = b
                img.draw_rectangle(right_max_b[0:4], color = (255, 0, 0))
                img.draw_cross(right_max_b[5], right_max_b[6])
        if black_blobs_center:
            for b in black_blobs_center:
                center_max_b = black_blobs_center[0]
                if b.pixels() > center_max_b.pixels():
                    center_max_b = b
                img.draw_rectangle(center_max_b[0:4], color = (0, 255, 0))
                img.draw_cross(center_max_b[5], center_max_b[6])
        if black_blobs_front:
            for b in black_blobs_front:
                front_max_b = black_blobs_front[0]
                if b.pixels() > front_max_b.pixels():
                    front_max_b = b
                img.draw_rectangle(front_max_b[0:4])
                img.draw_cross(front_max_b[5], front_max_b[6])


        # 当前方区域和中间区域都有黑色色块时, 放物块
        if black_blobs_center:
            print("place ok!")
            mode = PLACE_MODE

        # 当两侧都有色块时, 前进并且旋转 
        elif black_blobs_left and black_blobs_right:
            forward_change_times += 1
            left_dy = measure.Get_dy(left_max_b)
            right_dy = measure.Get_dy(right_max_b)
            # 由面积差(有符号)确定旋转角度
            delta_area = left_max_b.pixels() - right_max_b.pixels()
            angle_degrees = int(delta_area / 300)
            forward_mm = measure.dy_to_forward((right_dy + left_dy) / 2)
            print("left and right:", angle_degrees, forward_mm)
            if angle_degrees >= 2 or angle_degrees <= -2: # 调整角度
                if left_right_change_times == 1:
                    uart.write(message.Steer_Command(angle_degrees))
                    print(1)
                    uart_time = time.time()
                else:
                    if uart.any():
                        command_receive = uart.readline().decode().strip()
                        if "steer" in command_receive:
                            uart.write(message.Steer_Command(angle_degrees))
                            print(1)
                            uart_time = time.time()
                        else:
                            continue
                    else:
                        delta_time = time.time() - uart_time
                        if delta_time > 2:
                            uart.write(message.Steer_Command(angle_degrees))
                            print(1)
                            uart_time = time.time()
                        else:
                            continue
            if forward_mm >= 5:
                if forward_change_times == 1:
                    uart.write(message.Shift_Command(forward_mm, 0))
                    uart_time = time.time()
                else:
                    if uart.any():
                        command_receive = uart.readline().decode().strip()
                        if "shift" in command_receive:
                            # print("uart write forward 2")
                            uart.write(message.Shift_Command(forward_mm, 0))
                            print("forward_mm", forward_mm)
                            uart_time = time.time()
                        else:
                            continue
                    else:
                        delta_time = time.time() - uart_time
                        if delta_time > 2:
                            uart.write(message.Shift_Command(forward_mm, 0))
                            print("forward_mm", forward_mm)
                            uart_time = time.time()
                        else:
                            continue

        # 当前方区域有色块, 左右其中一侧有色块, 前进且转动?
        elif black_blobs_front and black_blobs_left:
            # 由面积差(有符号)确定旋转角度
            delta_area = left_max_b.pixels()
            angle_degrees = int(delta_area / 300)
            front_dy = measure.Get_dy(front_max_b)
            left_dy = measure.Get_dy(left_max_b)
            forward_mm = measure.dy_to_forward((front_dy + left_dy) / 2)
            print("forward and left", angle_degrees, forward_mm)
            # 调整角度
            if angle_degrees >= 2 or angle_degrees <= -2:
                if left_right_change_times == 1:
                    uart.write(message.Steer_Command(angle_degrees))
                    uart_time = time.time()
                else:
                    if uart.any():
                        command_receive = uart.readline().decode().strip()
                        if "steer" in command_receive:
                            uart.write(message.Steer_Command(angle_degrees))
                            uart_time = time.time()
                        else:
                            continue
                    else:
                        delta_time = time.time() - uart_time
                        if delta_time > 2:
                            uart.write(message.Steer_Command(angle_degrees))
                            uart_time = time.time()
                        else:
                            continue
            # 前进
            if forward_mm >= 5:
                if forward_change_times == 1:
                    uart.write(message.Shift_Command(forward_mm, 0))
                    uart_time = time.time()
                else:
                    if uart.any():
                        command_receive = uart.readline().decode().strip()
                        if "shift" in command_receive:
                            # print("uart write forward 2")
                            uart.write(message.Shift_Command(forward_mm, 0))
                            uart_time = time.time()
                        else:
                            continue
                    else:
                        delta_time = time.time() - uart_time
                        if delta_time > 2:
                            uart.write(message.Shift_Command(forward_mm, 0))
                            uart_time = time.time()
                        else:
                            continue


        elif black_blobs_front and black_blobs_right:
            delta_area = - right_max_b.pixels()
            angle_degrees = int(delta_area / 300)
            front_dy = measure.Get_dy(front_max_b)
            right_dy = measure.Get_dy(right_max_b)
            forward_mm = measure.dy_to_forward((front_dy + right_dy) / 2)
            print("forward and right", angle_degrees, forward_mm)

            # 调整角度
            if angle_degrees >= 2 or angle_degrees <= -2:
                if left_right_change_times == 1:
                    uart.write(message.Steer_Command(angle_degrees))
                    uart_time = time.time()
                else:
                    if uart.any():
                        command_receive = uart.readline().decode().strip()
                        if "steer" in command_receive:
                            uart.write(message.Steer_Command(angle_degrees))
                            uart_time = time.time()
                        else:
                            continue
                    else:
                        delta_time = time.time() - uart_time
                        if delta_time > 2:
                            uart.write(message.Steer_Command(angle_degrees))
                            uart_time = time.time()
                        continue
            # 前进
            if forward_mm >= 5:
                if forward_change_times == 1:
                    uart.write(message.Shift_Command(forward_mm, 0))
                    uart_time = time.time()
                else:
                    if uart.any():
                        command_receive = uart.readline().decode().strip()
                        if "shift" in command_receive:
                            # print("uart write forward 2")
                            uart.write(message.Shift_Command(forward_mm, 0))
                            uart_time = time.time()
                        else:
                            continue
                    else:
                        delta_time = time.time() - uart_time
                        if delta_time > 2:
                            uart.write(message.Shift_Command(forward_mm, 0))
                            uart_time = time.time()
                        continue

        # 当前方区域有色块, 两侧无色块时, 前进?
        elif black_blobs_front:
            forward_change_times += 1
            front_dy = measure.Get_dy(front_max_b)
            forward_mm = measure.dy_to_forward(front_dy)
            print("forward", forward_mm)
            if forward_change_times == 1:
                uart.write(message.Shift_Command(forward_mm, 0))
                uart_time = time.time()
            else:
                if uart.any():
                    command_receive = uart.readline().decode().strip()
                    if "shift" in command_receive:
                        # print("uart write forward 2")
                        uart.write(message.Shift_Command(forward_mm, 0))
                        uart_time = time.time()
                    else:
                        continue
                else:
                    delta_time = time.time() - uart_time
                    if delta_time > 2:
                        uart.write(message.Shift_Command(forward_mm, 0))
                        uart_time = time.time()
                    else:
                        continue



        # 当前方区域无色块, 左右其中一侧有色块, 转动且前进
        elif black_blobs_right:
            delta_area = - right_max_b.pixels()
            angle_degrees = int(delta_area / 300)
            print("right", angle_degrees, 30)
            if left_right_change_times == 1:
                uart.write(message.Steer_Command(angle_degrees))
                uart_time = time.time()
            else:
                if uart.any():
                    command_receive = uart.readline().decode().strip()
                    if "steer" in command_receive:
                        uart.write(message.Steer_Command(angle_degrees))
                        uart_time = time.time()
                else:
                    delta_time = time.time() - uart_time
                    if delta_time > 2:
                        uart.write(message.Steer_Command(angle_degrees))
                        uart_time = time.time()

            if forward_change_times == 1:
                uart.write(message.Shift_Command(30, 0))
                uart_time = time.time()
            else:
                if uart.any():
                    command_receive = uart.readline().decode().strip()
                    if "shift" in command_receive:
                        # print("uart write forward 2")
                        uart.write(message.Shift_Command(30, 0))
                        uart_time = time.time()
                    else:
                        continue
                else:
                    delta_time = time.time() - uart_time
                    if delta_time > 2:
                        uart.write(message.Shift_Command(30, 0))
                        uart_time = time.time()
                    else:
                        continue


        elif black_blobs_left:
            delta_area = left_max_b.pixels()
            angle_degrees = int(delta_area / 300)
            print("left", angle_degrees, 30)
            if left_right_change_times == 1:
                uart.write(message.Steer_Command(angle_degrees))
                uart_time = time.time()
            else:
                if uart.any():
                    command_receive = uart.readline().decode().strip()
                    if "steer" in command_receive:
                        uart.write(message.Steer_Command(angle_degrees))
                        uart_time = time.time()
                else:
                    delta_time = time.time() - uart_time
                    if delta_time > 2:
                        uart.write(message.Steer_Command(angle_degrees))
                        uart_time = time.time()
            if forward_change_times == 1:
                uart.write(message.Shift_Command(30, 0))
                uart_time = time.time()
            else:
                if uart.any():
                    command_receive = uart.readline().decode().strip()
                    if "shift" in command_receive:
                        # print("uart write forward 2")
                        uart.write(message.Shift_Command(30, 0))
                        uart_time = time.time()
                    else:
                        continue
                else:
                    delta_time = time.time() - uart_time
                    if delta_time > 2:
                        uart.write(message.Shift_Command(30, 0))
                        uart_time = time.time()
                    else:
                        continue

        # 当前方和左右无色块, 返回错误信息?
        else:
            uart.write(message.Forward_Command(PLACE_FAILED))
            print("failed")


    if mode == PLACE_MODE:
        #print("place")
        Servo_Control(4)
        time.sleep(3)
        uart.write(message.Steer_Command(0))
        uart.write(message.Shift_Command(0, 0))
        uart.write(message.Forward_Command(PLACE_SUCCESS))
        #mode = PLACE_PREPARE_MODE



