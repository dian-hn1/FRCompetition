#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 这是使用法奥api进行路径规划的demo，可以直接运行

import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
# from serial.tools import list_ports
# import serial
import rospy
import time
from math import sin, cos, pi
from chemistryexp import HNchemistry
import struct

############################## 预设全局变量 ##################################
'''
常用变量
'''
eP1 = [0.000, 0.000, 0.000, 0.000]
dP1 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
oP1 = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
a_bias = 220.0
b_bias = 150.0

'''
仪器区的初始位置
'''
solid_beaker_pos0 = [0.000, -700.000]    # X -1.888 Y -541.005 Z 71.539 RX 89.704 RY 0.519 RZ 2.157
solid_beaker_pos1 = [0.000, -500.000]  
pour_pos = [0.000, -400.000]
funnel_pos = []

'''
关键姿势的J与P，做伺服���位用
'''


# 初始化化学实验对象
def init():
    global fr5_A
    global fr5_B
    fr5_A = HNchemistry(1)
    fr5_B = HNchemistry(2)
    fr5_A.dou_go_start(fr5_B)

if __name__ == "__main__":
    ############################# 开始 ###############################
    print("---------------FR5机械臂化学协作实验------------------\n") 
    init()
    fr5_A.pick(solid_beaker_pos0, "xp", 2)
    # fr5_A.MoveL(0.000, -600.000, 100.000)
    # fr5_A.MoveLDelta(0.000, 100.000, 0.000)
    # fr5_A.point_safe_move(pour_pos)
    fr5_A.pour(5, 20, pour_pos, "xp", 2)
    fr5_A.put(solid_beaker_pos1, "xp", 2)
    
    # ################### FR5B去仪器区分别抓取试管和烧杯 ########################
    # string = ' '.join(map(str, liangtong_xy_test))
    # fr5_B.F101_catch_02(string, "xn", "3", False, is_display=False)

    # # FR5A称量液体
    # fr5_A.weight_liquid(fr5_B, 500)
    
    # 其他实验步骤（已注释）
    # fr5_A.Add_KMnO4(fr5_B)
    # fr5_B.pour(10.0, 10.0, -2, 100)
    # fr5_A.dou_go_start(fr5_B)
    # array = [550, 0]
    # string = ' '.join(map(str, liangtong_xy_input))
    # fr5_B.F101_catch_02(string, "xp", "2", False, is_display=False)
    # fr5_B.MoveL(0.0, 0.0, 300.0)
    # p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
    # while(type(p3_auto_weight) != tuple):
    #     p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
    #     time.sleep(0.1)
    # p3_auto_weight = p3_auto_weight[1]
    # fr5_B.robot.MoveJ(j3_auto_weight, 0, 0, p3_auto_weight, 20.0, 0, 100.0, eP1, -1.0, 0, oP1)
    # time.sleep(2)
    # string = ' '.join(map(str, liangtong_xy_output))
    # fr5_B.F101_put_02(string, "xn", "2", False, is_display=False)
    # fr5_B.MoveL(0.0, 0.0, 300.0)
    # fr5_B.Go_to_start_zone()
    # fr5_A.add(500.0)
