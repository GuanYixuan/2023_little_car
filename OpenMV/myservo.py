import sensor, image, time
from pyb import Pin, Timer, UART
from pyb import Servo

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) #320*240
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) #关闭自动增益
sensor.set_auto_whitebal(False) #关闭白平衡


tim = Timer(4, freq = 50)
ch1 = tim.channel(1, Timer.PWM, pin = Pin('P7')) # 开合舵机: 张开: 10 闭合: 7
ch2 = tim.channel(2, Timer.PWM, pin = Pin('P8')) # 俯仰舵机: 水平: 4 运输: 5 检测: 7

def Servo_Control(mode):

    if mode == 0: # 找物块
        ch1.pulse_width(3500) # 张开
        ch2.pulse_width_percent(4)

    elif mode == 1: # 夹物块
        time.sleep_ms(200)
        ch1.pulse_width(2425) # 闭合
        ch2.pulse_width_percent(4)

    elif mode == 2: # 检测物块
        time.sleep_ms(200)
        ch1.pulse_width(2425) # 闭合
        ch2.pulse_width_percent(6)

    elif mode == 3: # 夹持物块移动
        ch2.pulse_width_percent(8)
        time.sleep_ms(200)
        ch1.pulse_width(2425) # 闭合

    elif mode == 4: # 放置物块 放下-张开-抬起-发送指令-自由模式
        ch2.pulse_width_percent(5)
        time.sleep_ms(400)
        ch1.pulse_width(3500) # 张开
        time.sleep_ms(400)

    elif mode == 5: # 放置物块后的过渡模式
        ch1.pulse_width(3500) # 张开
        ch2.pulse_width_percent(10)
        time.sleep(8)
        ch2.pulse_width_percent(4)

    elif mode == 6: # FREE_MODE
        ch1.pulse_width(3500) # 张开
        ch2.pulse_width_percent(10)

# while(True):
#     img = sensor.snapshot()
#     ch2.pulse_width_percent(8)
#     #Servo_Control(6)
#     #ch2.pulse_width_percent(4)
#     #ch1.pulse_width(2425) # 闭合
#     time.sleep(2)
    #Servo_Control(3)
    #time.sleep(2)
