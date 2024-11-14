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
import threading
import rospy

# 添加父目录到sys.path
sys.path.append('../')
parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_path)

from scripts.basic_operation import fr5robot

##############################预设全局变量##################################
'''
常用变量
'''
eP1 = [0.000, 0.000, 0.000, 0.000]
dP1 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
oP1 = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
a_bias = 220.0
b_bias = 150.0
base_height = 0

'''
仪器区的初始位置
'''


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
        sel_num:抓取的对: 1---试管 2---烧杯 3---量筒 4---反应瓶 5---引导器 6---漏斗
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
            elif int(sel_num) == 6:
                start_catch_position += [160.0]
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
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(0.0, -100.0, 0.0)
            time.sleep(1)
        elif catch_direction == "xn":
            position1 = start_catch_position
            position1[0] += 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(-100, 0.0, 0.0)
            time.sleep(1)
        elif catch_direction == "xp":
            position1 = start_catch_position
            position1[0] -= 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(100, 0.0, 0.0)
            time.sleep(1)
        elif catch_direction == "yp":
            position1 = start_catch_position
            position1[1] -= 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(0.0, 100.0, 0.0)
            time.sleep(1)

        time.sleep(1)

        if int(sel_num) == 4:
            self.MoveGripper(1, 0, 50, 80, 10000, 1)  # close
            time.sleep(3)
        else:
            self.MoveGripper(1, 0, 50, 80, 10000, 1)  # close
            time.sleep(3)

        if is_display:
            self.MoveLDelta(0.0, 0.0, 20.0, 20.0)
            time.sleep(1)
            self.MoveLDelta(0.0, 0.0, -20.0, 20.0)
            time.sleep(1)
            self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
            time.sleep(3)
            self.MoveLDelta(0.0, 0.0, 50.0, 20.0)
            self.Go_to_start_zone()
        elif int(sel_num) == 5:
            self.MoveLDelta(0.0, 0.0, 200.0, 20.0)
        else:
            self.MoveLDelta(0.0, 0.0, 20.0, 20.0)
            time.sleep(1)

        print("pick动作完成")

    def put(self, start_catch_position, catch_direction, sel_num, is_force_sensor=False, obj_height=base_height):
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
            elif int(sel_num) == 6:
                start_catch_position += [160.0]
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
            self.MoveLDelta(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)
        elif catch_direction == "xn":
            position1 = start_catch_position
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)
        elif catch_direction == "xp":
            position1 = start_catch_position
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)
        elif catch_direction == "yp":
            position1 = start_catch_position
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0 + obj_height)
            time.sleep(1)

        time.sleep(1)

        if int(sel_num) == 4:
            self.MoveGripper(1, 50, 50, 10, 10000, 1)  # Open
            time.sleep(3)
        else:
            self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
            time.sleep(3)
            
        self.MoveLDelta(0.0, 0.0, 300.0)

        print("put动作完成")

