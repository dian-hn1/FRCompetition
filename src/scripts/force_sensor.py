#!/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
import frrpc
from std_msgs.msg import UInt8MultiArray,Float32MultiArray,String
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import socket
import keyboard
import sys, select, termios, tty

# from moveit_msgs.msg import joint_pos

# 力传感器数据，分别为xyz的力（单位：N），xyz的力矩（单位：Nm）
force_data = [1.0,0.0,0.0,0.0,0.0,0.0]
# 力传感器首次采集数据，分别为xyz的力（单位：N），xyz的力矩（单位：Nm）
sensor_offset = [0.0,0.0,0.0,0.0,0.0,0.0]
# 力传感器校准后开始产生的力的数据，分别为xyz的力（单位：N），xyz的力矩（单位：Nm）
real_force_data = [0.0,0.0,0.0,0.0,0.0,0.0]

def zero_calibration():
    '''
    对传感器数据进行归零，得到真实末端的力各种数据
    '''
    global sensor_offset , real_force_data , force_data
    print('forcedata:=======',force_data)
    sensor_offset = force_data # 计算偏移量
    for i in range(6):
        real_force_data[i] = force_data[i] - sensor_offset[i] # 初次归零
    print("起始末端的力/力矩数据为：",real_force_data)
    print("起始末端的力/力矩offset数据为：",sensor_offset)

def read_force_sensor():
    global real_force_data,sensor_offset
    for i in range(6):
        real_force_data[i] = force_data[i] - sensor_offset[i] # 更新数据
        real_force_data[i] = f"{real_force_data[i]:.2f}"
    print("末端的z方向力数据为：",real_force_data[2])

def force_callback(force_msg):
    global force_data 
    force_data= force_msg.data
    # print(force_data)

def talker():
    # rospy.init_node('force_publisher', anonymous=True)
    rospy.Subscriber('/force_msg',Float32MultiArray,force_callback)
    # while not rospy.is_shutdown():
    #     pass
    # time.sleep(2)
    # force_max = 1.0
    # is_to_limit = True
    # zero_calibration()
    # while is_to_limit :
    #     read_force_sensor()
    #     pass
    #     if float(real_force_data[2]) > 1.0:
    #         zero_calibration() # 数据漂的严重，重新校准
    #     if float(real_force_data[2]) < force_max*(-1.0) :
    #         print("超过材料受力上限！")
    #         is_to_limit = False
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print(e,"\n")
 