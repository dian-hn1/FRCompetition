#!/usr/bin/env python
# -*- coding: utf-8 -*-

#这是使用法奥api进行路径规划的demo，可以直接运行

import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
from serial.tools import list_ports
import serial

import rospy
import time
from math import sin, cos,pi
from chemistryexp import HNchemistry
import numpy
import struct
# from serial.tools import list_ports
# import serial
##############################预设全局变量##################################
'''
常用变量
'''
eP1=[0.000,0.000,0.000,0.000]
dP1=[1.000,1.000,1.000,1.000,1.000,1.000]
oP1=[0.000,0.000,0.000,0.000,0.000,0.000]
a_bias = 220.0
b_bias = 150.0
'''
仪器区的初始位置
'''
liangtong_xy_input = [600, 0] # 用于称量液体浓盐酸的量筒，对于B
shaobei_xy_input = [600, -100] # 用于称量固体C的烧杯，对于B
sanjing_xy_input = [520, -300] # 用于反应容器的三颈烧瓶，对于B
liangtong_xy_output = [-500, -100]
shaobei_xy_output = [-500, -300]
liangtong_xy_test = [-500, -300]
liquid_weight_pos = [-700, -300, 15]


'''
关键姿势的J与P，做伺服到位用
'''
j1_auto_weight = [-36.870, -59.151, -137.756, -163.093, -126.870, 0.000]
j2_auto_weight = [40.542, -57.601, -138.394, -164.005, -49.458, 0.000]
j3_auto_weight = [-64.941, -59.151, -137.756, -163.094, 25.056,0.001]
j4_auto_weight = [-49.458, -55.125, -124.682, -180.193, -49.458, 0.000]# 这个是机械臂复位的J
j5_auto_weight = [-119.608, -101.188, -110.403, -148.409, -119.608, 0.000]# 这是机械臂A要靠过来抓三颈烧瓶时的先验关节角度

p6_auto_weight = [320.0, -340.0, 400.0, 90.0, 0.0 ,0.0]

# p1_auto_weight = []
# fr5_A = HNchemistry(1)
# fr5_B = HNchemistry(2)

def init():
    global fr5_A
    global fr5_B
    fr5_A = HNchemistry(1)
    fr5_B = HNchemistry(2)
    # time.sleep(1)
    fr5_A.dou_go_start(fr5_B)




if __name__ == "__main__":
    #############################开始###############################
    print("---------------FR5机械臂化学协作实验------------------\n") 
    init()
    ###################FR5B去仪器区分别抓取试管和烧杯########################
    # input("按任意键继续")
    string = ' '.join(map(str, liangtong_xy_test))
    fr5_B.F101_catch_02(string, "xn" , "3" , False, is_display = False)

    fr5_A.weight_liquid(fr5_B, 500)
    
    # fr5_A.Add_KMnO4(fr5_B)
    
    
    # fr5_B.pour(10.0, 10.0, -2, 100)

    # fr5_A.dou_go_start(fr5_B)
    # array = [550, 0]
    
    
    

    # string = ' '.join(map(str, liangtong_xy_input))
    # fr5_B.F101_catch_02(string, "xp" , "2" , False, is_display = False)
    # fr5_B.MoveL(0.0, 0.0, 300.0)
    # p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
    # while( type(p3_auto_weight) != tuple):
    #         p3_auto_weight = fr5_B.robot.GetForwardKin(j3_auto_weight)
    #         time.sleep(0.1)
    # p3_auto_weight = p3_auto_weight[1]
    # fr5_B.robot.MoveJ(j3_auto_weight,0,0,p3_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
    # time.sleep(2)
    # string = ' '.join(map(str, liangtong_xy_output))
    # fr5_B.F101_put_02(string, "xn" , "2" , False, is_display = False)
    # fr5_B.MoveL(0.0, 0.0, 300.0)
    # fr5_B.Go_to_start_zone()
    # fr5_A.add(500.0)
    