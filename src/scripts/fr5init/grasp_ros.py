#!/usr/bin/env python
# -*- coding: utf-8 -*-

#这是使用法奥api进行路径规划的demo，可以直接运行
import sys
sys.path.append('/home/zjh/HN-1/demo_ws/src/frcobot_ros/fr5_moveit_config/')

import os
# from kyle_robot_toolbox.camera import Camera

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import rospy
import time
from fr5_init_new import fr5robot
import numpy as np
import random
import vision_arm_test
import tf_eyeoffhand

##############################预设全局变量##################################
'''
常用变量
'''
eP1=[0.000,0.000,0.000,0.000]
dP1=[1.000,1.000,1.000,1.000,1.000,1.000]
oP1=[0.000,0.000,0.000,0.000,0.000,0.000]
a_bias = 220.0
b_bias = 150.0

J_init = [-70.423, -59.731, -108.197, -103.020, 90.319, 18.574]
J_end = [16.831, -81.841, -108.913, -78.684, 89.830, 16.830]

obj_transform_mat = [[1.0, 0.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0], 
                     [0.0, 0.0, 0.0, 1.0]]

def add_noise(range1, gaussian=False):
    noiselist = []
    for i in range(6):
        if gaussian:
            angle = np.clip(np.random.normal(0, 1) * range1, -range1, range1)
        else:
            angle += random.uniform(-5, 5)
        noiselist.append(angle)
    return noiselist

# use new sdk
if __name__ == '__main__':
    # init fr5B
    fr5_B = fr5robot(2)
    P_init = fr5_B.robot.GetForwardKin(J_init)
    while( type(P_init) != tuple):
            P_init = fr5_B.robot.GetForwardKin(J_init)
            time.sleep(0.1)
    P_init = P_init[1]
    fr5_B.robot.MoveJ(J_init,0,0,P_init,20.0,0,100.0,eP1,-1.0,0,oP1)
    time.sleep(2)

    while True:
        # input('Press Enter to continue next exp: ')
        # update obj_transform_mat
        level = input('Please enter the level of noise: ')
        print('cup loc:',vision_arm_test.cup_loc_to_base)
        # print('noise:',level)
        level = float(level)
        noise = add_noise(range1=level,gaussian=True)
        
        
        input('Press Enter to continue next exp: ')
        fr5_B.robot.MoveCart([vision_arm_test.cup_loc_to_base[0], vision_arm_test.cup_loc_to_base[1] , 150.0+ b_bias, 179.0, 0.0, -179.0],0,0)
        time.sleep(2)
        # fr5_B.robot.MoveCart([vision_arm_test.cup_loc_to_base[0], vision_arm_test.cup_loc_to_base[1] , 15+b_bias, 179.0, 0.0, -179.0],0,0)
        # time.sleep(2)
        
        ########################添加一段噪声再执行##########################
        fr5_B_end = [vision_arm_test.cup_loc_to_base[0], vision_arm_test.cup_loc_to_base[1] , 20+b_bias, 179.0, 0.0, -179.0]
        # 计算该期望末端位置逆运动学解算为关节角度
        fr5_B_joint = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
        while( type(fr5_B_joint) != tuple):
                fr5_B_joint = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                time.sleep(0.008)
        fr5_B_joint = fr5_B_joint[1]
        print('fr5_B_jointpos_old: ',fr5_B_joint)
        
        # 添加噪声
        fr5_B_joint = [fr5_B_joint[i] + noise[i] for i in range(6)]
        print('fr5_B_joint: ',fr5_B_joint)
        fr5_B_pos = fr5_B.robot.GetForwardKin(fr5_B_joint)
        while( type(fr5_B_pos) != tuple):
                fr5_B_pos = fr5_B.robot.GetForwardKin(fr5_B_joint)
                time.sleep(0.1)
        fr5_B_pos = fr5_B_pos[1]
        # 执行servoJ
        fr5_B.robot.MoveJ(fr5_B_joint,0,0,fr5_B_pos,20.0,0,100.0,eP1,-1.0,0,oP1)
        
        
        
        fr5_B.robot.MoveGripper(1, 0, 50, 10, 10000, 1)
        time.sleep(3)
        # fr5_B.MoveL(0.0,0.0,300.0)
        P_init = fr5_B.robot.GetForwardKin(J_init)
        while( type(P_init) != tuple):
                P_init = fr5_B.robot.GetForwardKin(J_init)
                time.sleep(0.1)
        P_init = P_init[1]
        fr5_B.robot.MoveJ(J_init,0,0,P_init,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)
        P_end = fr5_B.robot.GetForwardKin(J_end)
        while( type(P_end) != tuple):
                P_end = fr5_B.robot.GetForwardKin(J_end)
                time.sleep(0.1)
        P_end = P_end[1]
        fr5_B.robot.MoveJ(J_end,0,0,P_end,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(5)
        fr5_B.robot.MoveGripper(1, 100, 50, 10, 10000, 1)
        time.sleep(3)
        P_init = fr5_B.robot.GetForwardKin(J_init)
        while( type(P_init) != tuple):
                P_init = fr5_B.robot.GetForwardKin(J_init)
                time.sleep(0.1)
        P_init = P_init[1]
        fr5_B.robot.MoveJ(J_init,0,0,P_init,20.0,0,100.0,eP1,-1.0,0,oP1)
        time.sleep(2)





