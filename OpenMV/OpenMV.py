# Untitled - By: Lenovo - 周一 8月 28 2023

import sensor, image, time
from pyb import UART
from pyb import Servo
import math
import measure
import message
from wait import wait_uart
from myservo import Servo_Control

# 颜色阈值
green_threshold  = (26, 62, -42, -14, 12, 33)
orange_threshold = (20, 90, 15, 60, 20, 100)
yellow_threshold = (39, 85, -27, 6, 32, 90) #黄色物块
black_threshold = (0, 22, -128, 125, -2, 127) # 家
red_threshold = (2, 51, 15, 60, 9, 70)
blue_threshold = (3, 24, -28, 31, -44, 0)


# 串口定义
uart = UART(3, 9600, timeout = 10000, timeout_char=10000)

# 舵机定义
#open_servo = Servo(1)
#up_down_servo = Servo(2)
#GRASP_ANGLE = -55
#PLACE_ANGLE = 0
#ARMDOWN_ANGLE = -60
#ARMUP_ANGLE = -20
#ARMMID_ANGLE = -30
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
CHECK_MODE = 5 # 检查是否抓到物块
TRANSPORT_MODE = 6 # 运输物块
FREE_MODE = 7 # 空闲模式
TRANSITION_MODE = 8 # 放完物块的过渡模式

# 信号定义
GRAB_SUCCESS = 0
GRAB_FAILED = 1
PLACE_SUCCESS = 2
PLACE_FAILED = 3

GRAB_SUCCESS_CONTENT: str = "GRSUCC\n"
GRAB_FAILED_CONTENT: str = "GRFAIL\n"
PLACE_SUCCESS_CONTENT: str = "PLSUCC\n"
PLACE_FAILED_CONTENT: str = "PLFAIL\n"

# 等待时间
WAIT_TIME_MAX = 2

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

# 夹持控制
def Servo_Init():
    open_servo.angle(PLACE_ANGLE)
    up_down_servo.angle(ARMDOWN_ANGLE)


Sensor_Init()
clock = time.clock() # 追踪频率

# 初始化
Servo_Control(6)
#time.sleep(3)
shift_times = 0

left_right_aim_times = 0
forward_aim_times = 0
change_times = 0

up_roi = (138, 0, 70, 134)
home_center_roi = (130, 103, 110, 81)
home_left_roi = (40, 0, 100, 200)
home_right_roi = (240, 0, 80, 200)
home_front_roi = (130, 1, 100, 74)
home_center_front_roi = (150, 57, 48, 62)
wall_front_roi = (0, 0, 321, 100)



uart_time = time.time()
place_time = 0
mode = FREE_MODE
# test
#mode = GRASP_PREPARE_MODE
#mode = PLACE_PREPARE_MODE
#mode = TRANSITION_MODE
place_begin_time = time.time()

f = open("test.txt", "w")
# 主循环
#with open("F:\\test.txt", "w") as f:
while(True):
    f.flush()
    clock.tick()
    img = sensor.snapshot()

    # 运输模式
    if mode == TRANSPORT_MODE:
        Servo_Control(3)
        if uart.any():
            command_receive = uart.readline().decode().strip()
            if command_receive == "GRASP":
                change_times = 0
                mode: int = GRASP_PREPARE_MODE
                print("进入抓取准备")
                f.write("进入抓取准备\n")

            elif command_receive == "PLACE":
                change_times = 0
                place_begin_time = time.time()
                mode: int = PLACE_PREPARE_MODE
                print("进入放置准备")
                f.write("进入放置准备0\n")

        continue

    # 空闲模式
    if mode == FREE_MODE:
        Servo_Control(6)
        # test
        #time.sleep(3)
        #mode = GRASP_PREPARE_MODE
        if uart.any():
            command_receive = uart.readline().decode().strip()
            if command_receive == "GRASP":
                change_times = 0
                mode: int = GRASP_PREPARE_MODE
                print("进入抓取准备")
                f.write("进入抓取准备\n")
            elif command_receive == "PLACE":
                change_times = 0
                place_begin_time = time.time()
                mode: int = PLACE_PREPARE_MODE
                print("进入放置准备")
                f.write("进入放置准备\n")
        continue

    # 抓取部分
    # 抓取准备模式
    if mode == GRASP_PREPARE_MODE:
        Servo_Control(0)
        #img = img.binary([orange_threshold], invert=True, zero=True)  # 1 二值化
        blobs = img.find_blobs([red_threshold, yellow_threshold, orange_threshold], merge=False) # 2 找色块
        if blobs:
            # 找出像素面积最大的物块
            max_b = blobs[0]
            for b in blobs:
                if b.pixels() >= max_b.pixels():
                    max_b = b
            img.draw_rectangle(max_b[0:4])
            img.draw_cross(max_b[5], max_b[6])
            print("进入瞄准")
            f.write("进入瞄准\n")
            mode = AIM_MODE

        else:
            command = message.Forward_Command(GRAB_FAILED)
            uart.write(command)
            print("未找到物块: 抓取失败") # :bug!尝试抓一两次
            f.write("未找到物块: 抓取失败\n")
            mode = FREE_MODE


    # 瞄准模式 # TODO: 此处比较慢
    if mode == AIM_MODE:
        Servo_Control(0)
        #img = img.binary([orange_threshold], invert=True, zero=True)  # 1 二值化
        blobs = img.find_blobs([red_threshold, yellow_threshold, orange_threshold], merge=False) # 2 找色块
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
            dy = measure.Get_dy(max_b)
            forward_mm = measure.dy_to_forward(dy)

            if forward_mm == 0:
                angle_degrees = 0
            if angle_degrees == 0:
                left_right_aim_times += 1                    # 角度瞄准次数+1
            else:
                left_right_aim_times = 0                     # 角度需要调整, 角度瞄准次数归零
                change_times += 1                 # 左右调整次数+1, 自进入瞄准状态后的第一次调整不需要等成功信息, 否则需要等成功信息
                if wait_uart(change_times, uart_time) == True:
                    uart.write(message.Steer_Command(angle_degrees))
                    print("角度调整: ", angle_degrees)
                    f.write("角度调整: \n")
                    uart_time = time.time()
                else:
                    continue

            if left_right_aim_times >= 5:                    # 角度瞄准五次后进行前后调整

                if forward_mm == 0:                          # 前后不需调整, 前后瞄准次数+1
                    forward_aim_times += 1
                else:
                    forward_aim_times = 0                    # 前后需要调整, 前后瞄准次数归零
                    change_times += 1
                    if wait_uart(change_times, uart_time) == True:
                        uart.write(message.Shift_Command(forward_mm, 0))
                        print("距离调整: ", forward_mm)
                        f.write("距离调整:\n")
                        uart_time = time.time()
                    else:
                        continue


                if forward_aim_times >= 5:
                    print("进入抓取模式")
                    f.write("进入抓取模式\n")
                    mode = GRASP_MODE


    if mode == GRASP_MODE:
        Servo_Control(1)
        #time.sleep(1)
        Servo_Control(2)
        print("进入检测模式")
        f.write("进入检测模式\n")
        mode = CHECK_MODE

    if mode == CHECK_MODE:
        print("抓取成功, 进入运输模式")
        #command = message.Forward_Command(GRAB_SUCCESS)
        #uart.write(command)
        #mode = TRANSPORT_MODE
        time.sleep_ms(500)
        img = sensor.snapshot()
        blobs = img.find_blobs([red_threshold, yellow_threshold, orange_threshold], roi= up_roi, pixels_threshold = 1200, merge=False)
        if blobs:
            for b in blobs:
                img.draw_rectangle(b[0:4])
                img.draw_cross(b[5], b[6])

            command = message.Forward_Command(GRAB_SUCCESS)
            uart.write(command)
            #Servo_Control(3)
            print("抓取成功, 进入运输模式")
            f.write("抓取成功, 进入运输模式\n")
            mode = TRANSPORT_MODE
            # test
            #mode = CHECK_MODE
        else:
            command = message.Forward_Command(GRAB_FAILED)
            uart.write(command)
            print("抓取失败, 进入自由模式") # :bug!尝试抓一两次
            f.write("抓取失败, 进入自由模式\n")
            mode = FREE_MODE
            # test
            #mode = CHECK_MODE



    if mode == PLACE_PREPARE_MODE:
        #time.sleep(3)
        #print("place prepare")
        Servo_Control(7)
        #img = sensor.snapshot()
        black_blobs_left = img.find_blobs([black_threshold], roi = home_left_roi, merge = True)
        black_blobs_right = img.find_blobs([black_threshold], roi = home_right_roi, merge = True)
        black_blobs_center = img.find_blobs([black_threshold], roi = home_center_roi, pixels_threshold = 800, merge = True)
        black_blobs_front = img.find_blobs([black_threshold], roi = home_front_roi, merge = True)
        black_blobs_center_front = img.find_blobs([black_threshold], roi = home_center_front_roi, pixels_threshold = 300, merge = True)
        blue_blobs = img.find_blobs([blue_threshold], roi = wall_front_roi, merge = True)

        for b in black_blobs_left:
            img.draw_rectangle(b[0:4], color = (255, 0, 0)) # rect
            img.draw_cross(b[5], b[6]) # cx, cy


        for b in black_blobs_right:
            img.draw_rectangle(b[0:4], color = (255, 0, 0)) # rect
            img.draw_cross(b[5], b[6]) # cx, cy

        for b_black in (black_blobs_center or black_blobs_center_front): # ? 语法
            img.draw_rectangle(b_black[0:4]) # rect
            img.draw_cross(b_black[5], b_black[6]) # cx, cy

        place_time = time.time() - place_begin_time


        print("place time", place_time)
        if place_time > 3:
            if wait_uart(change_times, uart_time) == True:
                uart.write(message.Shift_Command(0, 0))
                mode = PLACE_MODE

        if black_blobs_front:
            area_all_front = 0
            for b in black_blobs_front:
                area_all_front += b.pixels()
                img.draw_rectangle(b[0:4]) # rect
                img.draw_cross(b[5], b[6]) # cx, cy

            print("front", area_all_front)
            if area_all_front >= 6000:
                change_times += 1
                if wait_uart(change_times, uart_time) == True:
                    uart.write(message.Shift_Command(30, 0))
                    print("前方有色块, 前后调整", 30)
                    # f.write("两侧有色块, 前后调整\n")
                    uart_time = time.time()
                continue

        if (black_blobs_center or black_blobs_center_front) and blue_blobs:
            area_all_center = 0
            area_all_blue = 0
            for b_black in (black_blobs_center or black_blobs_center_front): # ? 语法
                area_all_center += b_black.pixels()
                img.draw_rectangle(b_black[0:4]) # rect
                img.draw_cross(b_black[5], b_black[6]) # cx, cys

            for b_blue in blue_blobs:
                area_all_blue += b_blue.pixels()
                img.draw_rectangle(b_blue[0:4]) # rect
                img.draw_cross(b_blue[5], b_blue[6]) # cx, cy

            print("center",area_all_center)
            print("blue", area_all_blue)
            if area_all_center >= 2500 and area_all_blue >= 18000:
                print("放置准备成功, 准备放置", area_all_center)
                change_times += 1
                if wait_uart(change_times, uart_time) == True:
                    uart.write(message.Shift_Command(0, 0))
                    mode = PLACE_MODE
                    uart_time = time.time()
                    continue

            elif area_all_blue < 18000:
                change_times += 1
                if wait_uart(change_times, uart_time) == True:
                    uart.write(message.Shift_Command(30, 0))
                    print("蓝色_前进", 30)
                    uart_time = time.time()
                    continue

        if black_blobs_right:
            area_all_right = 0
            for b in black_blobs_right:
                area_all_right += b.pixels()

            print("右转", area_all_right)
            if area_all_right > 10000:
                change_times += 1
                if wait_uart(change_times, uart_time) == True:
                    uart.write(message.Steer_Command(-10))
                    print("右转", 10)
                    uart_time = time.time()

        if black_blobs_left:
            area_all_left = 0
            for b in black_blobs_left:
                area_all_left += b.pixels()

            print("左转", area_all_left)
            if area_all_left > 10000:
                change_times += 1
                if wait_uart(change_times, uart_time) == True:
                    uart.write(message.Steer_Command(10))
                    print("左转", 10)
                    uart_time = time.time()


    if mode == PLACE_MODE:
        #print("place")
        print("放置成功, 进入空闲模式")
        uart.write(message.Forward_Command(PLACE_SUCCESS))
        Servo_Control(4)
        # f.write("放置成功, 进入空闲模式\n")
        mode = FREE_MODE
        #place_begin_time = time.time()
        #mode = PLACE_PREPARE_MODE

    if mode == TRANSITION_MODE:
        Servo_Control(5)
        mode = FREE_MODE

