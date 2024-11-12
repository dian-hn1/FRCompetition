#coding:UTF-8
'''
    HN代料模拟实验，包含六个流程的实现
    环境：Pytohn3.10 + ROS Noetic + Ubuntu20.04
    代码规范：变量为小写字母加下划线，函数为大写字母加下划线
    机器人：FR5
    创建日期：2024.04.16
    更新日期：2024.04.16,添加了化学实验的类，完成了基本骨架，zjh
            2024.04.29，补充了缺失的各个独立单元的函数体，zjh
            2024.04.29，提供了定点抓取，定点放置函数，自动称量函数写了一半，zjh
            2024.05.08,pour related functions are added, hcj
            2024.05.12，写了滴加KMnO4的大部分函数体，缺少蠕动泵电控和夹软管的操作，zjh
            2024.04.16，xxxxxx，xxx
            ...

'''
import sys
import os
import time
import copy
import math
import numpy as np
import threading
import rospy

# 添加父目录到sys.path
sys.path.append('../')
parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_path)

from fr5_init_new import fr5robot

##############################预设全局变量##################################
'''
常用变量
'''
eP1 = [0.000, 0.000, 0.000, 0.000]
dP1 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
oP1 = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
a_bias = 220.0
b_bias = 150.0
weight_height = 38.0

'''
仪器区的初始位置
'''
liangtong_xy_input = [600, 0]  # 用于称量液体浓盐酸的量筒，对于B
shaobei_xy_input = [600, -100]  # 用于称量固体C的烧杯，对于B
sanjing_xy_input = [520, -300]  # 用于反应容器的三颈烧瓶，对于B
liangtong_xy_output_B = [-500, -200]
shaobei_xy_output = [-500, -300]
sanjing_xy_output = [220, -560, 220]
liangtong_xy_used = [600, 0]
shaobei_xy_used = [600, -100]
yindaoqi_xy = [570, -300]
liangtong_xy_kmno4_givetoA = [-200, -700]  # 装有kmno4的量筒，需要递交给A，需要放在这个地方等待递交
liangtong_xy_kmno4_getfromB = [200, -600]
liangtong_xy_kmno4_puttopump = [-600, -300]
liquid_weight_pos = [-700, -300, 15]  # 液体称量三维坐标

'''
关键姿势的J与P，做伺服到位用
'''
j1_auto_weight = [-36.870, -59.151, -137.756, -163.093, -126.870, 0.000]
j2_auto_weight = [40.542, -57.601, -138.394, -164.005, -49.458, 0.000]
j3_auto_weight = [-64.941, -59.151, -137.756, -163.094, 25.056, 0.001]
j4_auto_weight = [-49.458, -55.125, -124.682, -180.193, -49.458, 0.000]  # 机械臂复位的J
j5_auto_weight = [-119.608, -101.188, -110.403, -148.409, -119.608, 0.000]  # 机械臂A抓三颈烧瓶的先验关节角度
j7_auto_weight = [-27.158, -90.814, -93.062, -174.026, -27.173, 0.384]  # 机械臂B抓引导器后放到水浴锅上方的过渡动作
j8_auto_weight = [-84.385, -71.425, -117.697, -170.878, 5.615, 0.000]

p6_auto_weight = [320.0, -340.0, 400.0, 90.0, 0.0, 0.0]

class HNchemistry(fr5robot):
    def __init__(self, index=1):
        super().__init__(index)
        self.robot.SetSpeed(60)

    def pick(self, start_catch_position, catch_direction, sel_num, is_force_sensor=False, is_display=False):
        '''
        指定坐标抓取指定物体,但是默认抓的高度可调，
        预期效果： 机械臂回到起始区————运动到指定位置抓取物体————抬起展示抓取效果————松开夹爪回到结束区
        start_catch_position: 目标物体绝对xy坐标
        catch_direction: yn——y轴负方向  xn——x负方向 yp xp
        sel_num:抓取的对: 1---试管 2---烧杯 3---量筒 4---反应瓶 5---引导器
        is_force_sensor:末端是否有力传感器
        is_display:是否为演示抓取，如果是，机械臂会自动复位
        '''
        if is_force_sensor:
            real_bias = a_bias
        else:
            real_bias = b_bias

        rxryrz = []
        if catch_direction == "yn":
            start_catch_position[1] += real_bias
            rxryrz = [90.0, 0.0, 0.0]
            dir = "yn"
        elif catch_direction == "xn":
            start_catch_position[0] += real_bias
            rxryrz = [90.0, 0.0, -90.0]
            dir = "xn"
        elif catch_direction == "xp":
            start_catch_position[0] -= real_bias
            rxryrz = [90.0, 0.0, 90.0]
            dir = "xp"
        elif catch_direction == "yp":
            start_catch_position[1] -= real_bias
            rxryrz = [90.0, 0.0, -179.9]
            dir = "yp"
        else:
            print("------error!-------")
            exit()

        if len(start_catch_position) != 2:
            print("xy坐标错误")
            exit()

        while True:
            if int(sel_num) == 1:
                start_catch_position += [130.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 2:
                start_catch_position += [65.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 3:
                start_catch_position += [65.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 4:
                start_catch_position += [168.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 5:
                start_catch_position += [150.0]
                start_catch_position += rxryrz
                break
            else:
                print("--------输入错误！请重新输入！---------")
                continue

        if int(sel_num) == 4:
            self.MoveGripper(1, 50, 50, 10, 10000, 1)  # close
            time.sleep(3)

        if catch_direction == "yn":
            position1 = start_catch_position
            position1[1] += 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveL(0.0, -100.0, 0.0)
            time.sleep(1)
        elif catch_direction == "xn":
            position1 = start_catch_position
            position1[0] += 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveL(-100, 0.0, 0.0)
            time.sleep(1)
        elif catch_direction == "xp":
            position1 = start_catch_position
            position1[0] -= 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveL(100, 0.0, 0.0)
            time.sleep(1)
        elif catch_direction == "yp":
            position1 = start_catch_position
            position1[1] -= 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveL(0.0, 100.0, 0.0)
            time.sleep(1)

        time.sleep(1)

        if int(sel_num) == 4:
            self.MoveGripper(1, 0, 50, 80, 10000, 1)  # close
            time.sleep(3)
        else:
            self.MoveGripper(1, 0, 50, 80, 10000, 1)  # close
            time.sleep(3)

        if is_display:
            self.MoveL(0.0, 0.0, 20.0, 20.0)
            time.sleep(1)
            self.MoveL(0.0, 0.0, -20.0, 20.0)
            time.sleep(1)
            self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
            time.sleep(3)
            self.MoveL(0.0, 0.0, 50.0, 20.0)
            self.Go_to_start_zone()
        elif int(sel_num) == 5:
            self.MoveL(0.0, 0.0, 200.0, 20.0)
        else:
            self.MoveL(0.0, 0.0, 20.0, 20.0)
            time.sleep(1)

        print("动作完成")

    def put(self, start_catch_position, catch_direction, sel_num, is_force_sensor=False, obj_height=weight_height):
        '''
        指定坐标放置指定物体，但是默认放置的高度可调，先写成这样吧，事情有点多，到时候再改
        预期效果： 机械臂回到起始区————运动到指定位置抓取物体————抬起展示抓取效果————松开夹爪回到结束区
        start_catch_position: 目标物体绝对xy坐标
        catch_direction: ym——y轴负方向  xm——x负方向
        sel_num:抓取的对: 1---试管 2---烧杯 3---量筒 4---反应瓶
        is_force_sensor:末端是否有力传感器
        height:桌子与机械臂基座的高度差
        obj_height:本次放置是要放到一个高度为objheight的东西上
        '''
        if is_force_sensor:
            real_bias = a_bias
        else:
            real_bias = b_bias

        rxryrz = []
        if catch_direction == "yn":
            start_catch_position[1] += real_bias
            rxryrz = [90.0, 0.0, 0.0]
            dir = "yn"
        elif catch_direction == "xn":
            start_catch_position[0] += real_bias
            rxryrz = [90.0, 0.0, -90.0]
            dir = "xn"
        elif catch_direction == "xp":
            start_catch_position[0] -= real_bias
            rxryrz = [90.0, 0.0, 90.0]
            dir = "xp"
        elif catch_direction == "yp":
            start_catch_position[1] -= real_bias
            rxryrz = [90.0, 0.0, -179.9]
            dir = "yp"
        else:
            print("------error!-------")
            exit()

        if len(start_catch_position) != 2:
            print("xy坐标错误")
            exit()

        while True:
            if int(sel_num) == 1:
                start_catch_position += [130.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 2:
                start_catch_position += [65.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 3:
                start_catch_position += [65.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 4:
                start_catch_position += [168.0]
                start_catch_position += rxryrz
                break
            elif int(sel_num) == 5:
                start_catch_position += [135.0]
                start_catch_position += rxryrz
                break
            else:
                print("--------输入错误！请重新输入！---------")
                continue

        if int(sel_num) == 4:
            pass

        if catch_direction == "yn":
            position1 = start_catch_position
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)
        elif catch_direction == "xn":
            position1 = start_catch_position
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)
        elif catch_direction == "xp":
            position1 = start_catch_position
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)
        elif catch_direction == "yp":
            position1 = start_catch_position
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveL(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)

        time.sleep(1)

        if int(sel_num) == 4:
            self.MoveGripper(1, 50, 50, 10, 10000, 1)  # Open
            time.sleep(3)
        else:
            self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
            time.sleep(3)

        print("动作完成")

