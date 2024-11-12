#!/bin/python3
# -*- coding: utf-8 -*-
import sys
sys.path.append("/home/wangzy/FR5_exp/fr5_gpt/src/scripts/")
import rospy
import time
import numpy as np
import vision_arm_test
import tf_eyeoffhand
from fr5_init_new import fr5robot
import joblib
import threading
from utils import tools
from PIL import Image, ImageDraw
import random

# 相机手眼标定的结果
# camera2base = [ 
#  [ 0.99885901,  0.02414404  ,0.04120379,99.50256158],
#  [ 0.02214817, -0.99859083 , 0.04822672,-542.01673407],
#  [ 0.04231012 ,-0.0472591 , -0.99798619,846.15889269],
#  [ 0.0,0.0,0.0,1.0]
# ]

camera2base = [[0.9985750339347319, 0.022189620209739077, 0.048533723916429926, 77.91185167719914], 
               [0.019898166421089946, -0.9986873577260162, 0.047197717013712184, -538.246214051216], 
               [0.049517315914004385, -0.046164729753085165, -0.9977057948871988, 840.3848447109431], 
               [0.0, 0.0, 0.0, 1.0]]

fr5_A = []
fr5_B = []
a_bias = 220.0
b_bias = 150.0

catch_xy = [0,0]
catch_xyz = [0,0,0]
previous_xyz = [0,0,0]
current_xyz = [0,0,0]
update_flag = 1
update_flagg = 1
cnt4servo = 17

def init():
    global fr5_A
    global fr5_B
    # fr5_A = fr5robot(1)
    fr5_B = fr5robot(2)
    # time.sleep(0.5)
    # fr5_A.dou_go_start(fr5_B)
    # fr5_B.dou_go_start(fr5_A)
    # fr5_A.robot.ServoMoveStart()
    ret = fr5_B.robot.ServoMoveStart()


def add_noise(range1, gaussian=False):
    noiselist = []
    for i in range(6):
        if gaussian:
            angle = np.clip(np.random.normal(0, 1) * range1, -range1, range1)
        else:
            angle += random.uniform(-5, 5)
        noiselist.append(angle)
    return noiselist

def IK_noise1(pos,robot,noise):
    '''
    为机械臂关节添加噪声，输入是一组期望位姿，输出是该位姿对应的一组关节角度，但是添加了噪声
    参数列表：
    @pos:期望位姿，数组形式
    @robot:机器人
    @noise:噪声，数组形式
    输出：
    @joint:加了噪声的关节角度
    '''
    joint = [0.,0.,0.,0.,0.,0.]
    pos_out = [0.,0.,0.,0.,0.,0.]
    joint_out = [0.,0.,0.,0.,0.,0.]
    # first_pos = [xx , yy + b_bias, zz,90.0, 0.0, 0.0]
    ret_j = robot.GetInverseKin(0,pos,-1)
    while( type(ret_j) != tuple):
        ret_j = robot.GetInverseKin(0,pos,-1)
        time.sleep(0.008)
    ret_j = ret_j[1]
    # 加噪声
    ret_j = [ret_j[i] + noise[i] for i in range(6)]
    # 这一行是玄学程序，没有必崩
    robot_pose = robot.GetActualToolFlangePose(0)
    # np.float转为float
    joint = [float(x) for x in ret_j]
    pos_out = robot.GetForwardKin(joint)
    while( type(pos_out) != tuple):
        pos_out = robot.GetForwardKin(joint)
        time.sleep(0.008)
    pos_out = pos_out[1]
    joint_out = joint
    print('添加一次噪声后：',pos_out)
    return pos_out, joint_out   
    
def vision_catch_cup(fr5_B,noise):
    global catch_xy
    if vision_arm_test.cup_position == [[],[],[]]:
        print("No cup detected.")
        pass
    else :
        print('enter this')
        vision_servo2(vision_arm_test.cup_loc_to_base,noise)
        time.sleep(0.001)

def vision_servo1(catch_xyz):
    '''
    视觉伺服部分代码
    设定：机械臂B，ym方向抓取物体，二维平面，不插值
    '''
    global previous_xyz
    global current_xyz
    global update_flag
    global update_flagg
    global cnt4servo
    current_time = time.time() # 获取当前时间
    # previous_xyz = catch_xyz# 当前帧的xyz坐标
    current_xyz = [0,0,0]
    x_flag = 1
    y_flag = 1

    if (update_flag == 1 and update_flagg == 1):
        previous_xyz = catch_xyz# 当前帧的xyz坐标

    # 每9帧更新一次
    while(cnt4servo %9 == 0):
        cnt4servo = 71
        current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
        current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
        current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
        update_flag = 0
        update_flagg = 1

    cnt4servo = cnt4servo -1
    if (update_flag == 1 and update_flagg == 1):
        update_flag = 1
        update_flagg = 0
    if(update_flag == 0 and update_flagg == 1):
        update_flag = 1
        update_flagg = 1

    # 当两帧内容不是[0,0,0]时，进行位置变化检测
    if(current_xyz[0] != 0 and current_xyz[1] != 0 and current_xyz[2] != 0 and previous_xyz[0] != 0 and previous_xyz[1] != 0 and previous_xyz[2] != 0):
        delta_xyz = [current_xyz[i] - previous_xyz[i] for i in range(3)] # 计算xyz坐标的变化量

        xyz_diff = np.linalg.norm(delta_xyz)  # 计算xyz坐标的总变化量

        change_threshold = 5  # 设定位置变化阈值
        current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
        current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
        current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
        if all(abs(delta) < change_threshold for delta in delta_xyz):# 如果xyz坐标的变化量小于阈值，认为目标物体稳定
            print("The target object is stable. Moving the robotic arm closer.")
            
            # 在这里编写靠近目标的代码

            # 先将夹爪xy移动到目标x，距离y100的位置，记得考虑sensor有无

            # 将末端z与目标z对齐，先按照桌面高度测试，后续可以根据实际情况调整

            # 开始伺服，缓慢向目标移动，计算当前夹爪与目标的差值，deltax与deltay，然后根据差值进行移动
            # print(type(current_xyz))
            fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
            while( type(fr5_B_end) != tuple):
                fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
                time.sleep(0.5)
            fr5_B_end = fr5_B_end[1]
            print("fr5_B_end:",fr5_B_end)
            # 对齐x
            x_flag = 1
            y_flag = 1
            # 这个功能保留一下，看后面怎么修改比较好
            # 如果末端的y值小于目标物体的y值，那么需要将末端回退
            # if (fr5_B_end[1] - b_bias - current_xyz < -0.0):
            #     # 更新物体坐标
            #     current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
            #     current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
            #     current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
                
            #     if((fr5_B_end[0] - current_xyz[0])>3.0 or (fr5_B_end[0] - current_xyz[0])<-3.0):
            #         y_flag = 0
            #         x_flag = 1
            #         # print('y_flag',y_flag)
            #     print('物体在夹爪后方，需要重新执行抓取')
            #     print("current_xyz[1]:",current_xyz[1])
            #     # 末端后退40mm
            #     for i in range(20):
            #         fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
            #         while( type(fr5_B_end_from_SDK) != tuple):
            #             fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
            #             time.sleep(0.008)
            #         print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
            #         fr5_B_end = fr5_B_end_from_SDK[1]
            #         print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz

            #         fr5_B_end[1] = fr5_B_end[1] + 2.0
            #         # 计算该期望末端位置逆运动学解算为关节角度
            #         fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
            #         while( type(fr5_B_jointpos_new) != tuple):
            #             fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
            #             time.sleep(0.008)
            #         fr5_B_jointpos_new = fr5_B_jointpos_new[1]
            #         # 执行servoJ
            #         fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
            #         time.sleep(0.008)
                # 末端复位，重新执行抓取


            while(x_flag):
                if fr5_B_end[0] - current_xyz[0] > 3.0:
                    print('执行x负方向不断跟进')
                    print("current_xyz[0]:",current_xyz[0])

                    fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                    while( type(fr5_B_end_from_SDK) != tuple):
                        fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                        time.sleep(0.008)
                    print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
                    fr5_B_end = fr5_B_end_from_SDK[1]
                    print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz

                    fr5_B_end[0] = fr5_B_end[0] - 2.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                    while( type(fr5_B_jointpos_new) != tuple):
                        fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                        time.sleep(0.008)
                    fr5_B_jointpos_new = fr5_B_jointpos_new[1]
                    # 执行servoJ
                    fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
                    current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
                    current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
                    current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
                    time.sleep(0.008)
                elif fr5_B_end[0] - current_xyz[0] < -3.0:
                    print('执行x正方向不断跟进')
                    print("current_xyz[0]:",current_xyz[0])

                    fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                    while( type(fr5_B_end_from_SDK) != tuple):
                        fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                        time.sleep(0.008)
                    print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
                    fr5_B_end = fr5_B_end_from_SDK[1]
                    print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz

                    fr5_B_end[0] = fr5_B_end[0] + 2.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                    while( type(fr5_B_jointpos_new) != tuple):
                        fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                        time.sleep(0.008)
                    fr5_B_jointpos_new = fr5_B_jointpos_new[1]
                    # 执行servoJ
                    fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
                    current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
                    current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
                    current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
                    time.sleep(0.008)
                else :
                    x_flag = 0
            # 如果x没对齐，优先对齐x
            if((fr5_B_end[0] - current_xyz[0])>3.0 or (fr5_B_end[0] - current_xyz[0])<-3.0):
                    y_flag = 0
                    x_flag = 1

            # 对齐y
            while(y_flag):
                if (fr5_B_end[1] - b_bias - current_xyz[1]) > 5.0:

                    current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
                    current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
                    current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])

                    if((fr5_B_end[0] - current_xyz[0])>3.0 or (fr5_B_end[0] - current_xyz[0])<-3.0):
                        y_flag = 0
                        x_flag = 1
 
                    print('执行y负方向不断跟进')
                    print("current_xyz[1]:",current_xyz[1])
                    print("current_xyz[0]:",current_xyz[0])

                    fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                    while( type(fr5_B_end_from_SDK) != tuple):
                        fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                        time.sleep(0.008)
                    print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
                    fr5_B_end = fr5_B_end_from_SDK[1]
                    print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz

                    fr5_B_end[1] = fr5_B_end[1] - 1.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                    while( type(fr5_B_jointpos_new) != tuple):
                        fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                        time.sleep(0.008)
                    fr5_B_jointpos_new = fr5_B_jointpos_new[1]
                    
                    # 执行servoJ
                    fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
                    time.sleep(0.008)

                elif (fr5_B_end[1] - b_bias - current_xyz[1]) < -5.0:

                    current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
                    current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
                    current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
                    
                    if((fr5_B_end[0] - current_xyz[0])>3.0 or (fr5_B_end[0] - current_xyz[0])<-3.0):
                        y_flag = 0
                        x_flag = 1
                        # print('y_flag',y_flag)
                    print('执行y正方向不断跟进')
                    print("current_xyz[1]:",current_xyz[1])

                    fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                    while( type(fr5_B_end_from_SDK) != tuple):
                        fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                        time.sleep(0.008)
                    print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
                    fr5_B_end = fr5_B_end_from_SDK[1]
                    print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz

                    fr5_B_end[1] = fr5_B_end[1] + 1.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                    while( type(fr5_B_jointpos_new) != tuple):
                        fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                        time.sleep(0.008)
                    fr5_B_jointpos_new = fr5_B_jointpos_new[1]
                    # 执行servoJ
                    fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
                    time.sleep(0.008)

                else:
                    y_flag = 0

    if ( x_flag == 0 and y_flag == 0):
        print("x,y对齐完成")
        fr5_B.robot.MoveGripper(1, 0, 50, 10, 10000, 1) # 夹爪闭合
        time.sleep(3)
        fr5_B.MoveL(0.0,0.0,50.0,40.0)
        time.sleep(2)
        fr5_B.MoveL(0.0,0.0,-50.0,40.0)
        time.sleep(2)
        fr5_B.robot.MoveGripper(1, 100, 50, 10, 10000, 1) # 夹爪张开
        time.sleep(3)
        fr5_B.Go_to_start_zone()
        exit()

                # bre/ak

        # else:# 如果xyz坐标的变化量大于阈值，认为目标物体移动
        #     print("The target object has moved. Aligning the robotic arm gripper.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            
            # 在这里编写朝向目标物体的代码

def vision_servo2(catch_xyz,noise):
    '''
    视觉伺服部分代码
    设定：机械臂B，ym方向抓取物体，二维平面，贝塞尔函数插值
    '''
    global previous_xyz
    global current_xyz
    global update_flag
    global update_flagg
    global cnt4servo
    current_time = time.time() # 获取当前时间
    # previous_xyz = catch_xyz# 当前帧的xyz坐标
    current_xyz = [0,0,0]
    x_flag = 1
    y_flag = 1

    if (update_flag == 1 and update_flagg == 1):
        previous_xyz = catch_xyz# 当前帧的xyz坐标

    # 每9帧更新一次
    while(cnt4servo %9 == 0):
        cnt4servo = 71
        current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
        current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
        current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
        update_flag = 0
        update_flagg = 1

    cnt4servo = cnt4servo -1
    if (update_flag == 1 and update_flagg == 1):
        update_flag = 1
        update_flagg = 0
    if(update_flag == 0 and update_flagg == 1):
        update_flag = 1
        update_flagg = 1

    # 当两帧内容不是[0,0,0]时，进行位置变化检测
    if(current_xyz[0] != 0 and current_xyz[1] != 0 and current_xyz[2] != 0 ):
        # delta_xyz = [current_xyz[i] - previous_xyz[i] for i in range(3)] # 计算xyz坐标的变化量

        # change_threshold = 5  # 设定位置变化阈值
        current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
        current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
        current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])

        # if all(abs(delta) < change_threshold for delta in delta_xyz):# 如果xyz坐标的变化量小于阈值，认为目标物体稳定
        print("The target object is stable. Moving the robotic arm closer.")
        
        fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
        while( type(fr5_B_end) != tuple):
            fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
            time.sleep(0.5)
        fr5_B_end = fr5_B_end[1]
        
        # fr5_B_end, fr5_B_joint = IK_noise()
        
        print("fr5_B_end:",fr5_B_end)
        print("current_xyz",current_xyz)
        # 如果x没对齐很严重，先插值
        # if(1>2):
        if(fr5_B_end[0] - current_xyz[0] > 20.0 or (fr5_B_end[0] - current_xyz[0] < -20.0)):
            ####################贝塞尔函数插值#######################
            print('=========================计算贝塞尔函数==============================')
            # 以末端为基准，考虑末端的运动
            start_point = [fr5_B_end[0], fr5_B_end[1]]# 运动起始时末端位置
            end_point = [current_xyz[0], (current_xyz[1] + b_bias)*1 ]# 运动结束时末端位置
            # 从start_point到end_point的距离，不考虑正负号，后面会兼容
            x_from_start2end = start_point[0] - end_point[0]# 可能是正值或负值，但在后面会被兼容
            y_from_start2end = start_point[1] - end_point[1]# 从二指中心到目标物体的y方向偏移量，可能是正值或负值，但在后面会被兼容
            control_point = [fr5_B_end[0] - x_from_start2end *0.95 , fr5_B_end[1] - y_from_start2end*0.05]# 计算出控制点
            print('x_from_start2end: {}, y_from_start2end: {}, control_point: {}'.format(x_from_start2end, y_from_start2end, control_point))
            # 生成曲线轨迹的数据点，在这里取至少30个点
            t = np.linspace(0, 1, max(int(abs(x_from_start2end)/0.5), 30))
            trajectory = [tools.bezier_curve(ti, start_point, control_point, end_point) for ti in t]
            # print('trajectory',trajectory)
            ########################################################
            
            ####################可视化轨迹测试########################
            # # 提取 x 坐标和 y 坐标
            # x_coords, y_coords = zip(*trajectory)

            # # print(x_coords,y_coords)
            # # 绘制曲线轨迹
            # # 计算最小和最大的 x 和 y 值
            # min_x = min(trajectory, key=lambda p: p[0])[0]
            # max_x = max(trajectory, key=lambda p: p[0])[0]
            # min_y = min(trajectory, key=lambda p: p[1])[1]
            # max_y = max(trajectory, key=lambda p: p[1])[1]
            # # 计算图像大小
            # width = max_x - min_x + 10
            # height = max_y - min_y + 10
            # image = Image.new("RGB", (int(width),int(height)), "white")
            # draw = ImageDraw.Draw(image)
            # # 平移坐标点以适应图像大小
            # translated_points = [(x - min_x + 5, y - min_y + 5) for x, y in trajectory]
            # draw.line(translated_points,fill="black",width=4)
            # image.show()
            ##########################################################

            # 跟随执行一半数量的轨迹点
            for i in range(int(len(trajectory)*0.5)):
                print('前往目标中...')
                print("current_xyz[0]:",current_xyz[0])
                
                
                # fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                # while( type(fr5_B_end_from_SDK) != tuple):
                #     fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                #     time.sleep(0.008)
                # fr5_B_end = fr5_B_end_from_SDK[1]
                
                
                # print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz
                # 从末端到目标位置做一个曲线插值
                fr5_B_end[0] = trajectory[i][0]
                fr5_B_end[1] = trajectory[i][1]
                fr5_B_end[2] = 190.0
                fr5_B_end[3] = 90.0
                fr5_B_end[4] = 0.0
                fr5_B_end[5] = 0.0
                # 计算该期望末端位置逆运动学解算为关节角度
                fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                while( type(fr5_B_jointpos_new) != tuple):
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                    time.sleep(0.008)
                fr5_B_jointpos_new = fr5_B_jointpos_new[1]
                
                # noise = add_noise(range1=1,gaussian=True)
                ret_j = [fr5_B_jointpos_new[i] + noise[i] for i in range(6)]
                # 这一行是玄学程序，没有必崩
                # robot_pose = robot.GetActualToolFlangePose(0)
                # np.float转为float
                joint = [float(x) for x in ret_j]
                # 执行servoJ
                fr5_B.robot.ServoJ(joint,0.0,0.0,0.016,0.0,0.0)
                # fr5_B.robot.MoveJ(joint,0,0)
                time.sleep(0.016)
        
        # 如果x没对齐不是很严重，这个时候只需要在x轴上移动就行了
        if( ((fr5_B_end[0] - current_xyz[0] < 20.0) and (fr5_B_end[0] - current_xyz[0] > 10.0)) ):
            print('执行x负方向不断跟进')
            print("current_xyz[0]:",current_xyz[0])

            delta = abs(fr5_B_end[0] - current_xyz[0])
            fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
            while( type(fr5_B_end_from_SDK) != tuple):
                fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                time.sleep(0.008)
            # print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
            fr5_B_end = fr5_B_end_from_SDK[1]
            print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz
            fr5_B_end[2] = 190.0
            fr5_B_end[3] = 90.0
            fr5_B_end[4] = 0.0
            fr5_B_end[5] = 0.0
            # 下一个末端位置
            # fr5_B_end[0] = fr5_B_end[0] - 5.0
            # 计算该期望末端位置逆运动学解算为关节角度joint3
            fr5_B_joint3 = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
            while( type(fr5_B_joint3) != tuple):
                fr5_B_joint3 = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                time.sleep(0.008)
            fr5_B_joint3 = fr5_B_joint3[1]

            # 添加误差
            fr5_B_joint1 = [fr5_B_joint3[i] + noise[i] for i in range(6)]
            fr5_B_joint1 = [float(x) for x in fr5_B_joint1]
            # 正解成pos
            pos_out = fr5_B.robot.GetForwardKin(fr5_B_joint1)
            while( type(pos_out) != tuple):
                pos_out = fr5_B.robot.GetForwardKin(fr5_B_joint1)
                time.sleep(0.008)
            pos_out = pos_out[1]
            print('不准的pos',pos_out)
            print('noise',noise)
            pos_out[0] = pos_out[0] - delta
            # 计算该期望末端位置逆运动学解算为关节角度joint2
            fr5_B_joint2 = fr5_B.robot.GetInverseKin(0,pos_out,-1)
            while( type(fr5_B_joint2) != tuple):
                fr5_B_joint2 = fr5_B.robot.GetInverseKin(0,pos_out,-1)
                time.sleep(0.008)
            fr5_B_joint2 = fr5_B_joint2[1]
            joint_out = [0.,0.,0.,0.,0.,0.]
            print('joint2=================',fr5_B_joint2)
            print('joint1=================',fr5_B_joint1)
            print('joint3=================',fr5_B_joint3)
            for i in range(6):
                joint_out[i] = fr5_B_joint2[i] - fr5_B_joint1[i] + fr5_B_joint3[i]
            print('joint_out=================',joint_out)
            # 执行servoJ
            # fr5_B.robot.ServoJ(fr5_B_joint2,0.0,0.0,0.016,0.0,0.0)
            fr5_B.robot.MoveJ(fr5_B_joint2,0,0)
            
            current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
            current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
            current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
            time.sleep(0.016)
        elif ((fr5_B_end[0] - current_xyz[0] < -10.0) and (fr5_B_end[0] - current_xyz[0] > -20.0)):
            print('执行x正方向不断跟进')
            print("current_xyz[0]:",current_xyz[0])

            delta = abs(fr5_B_end[0] - current_xyz[0])
            fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
            while( type(fr5_B_end_from_SDK) != tuple):
                fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                time.sleep(0.008)
            # print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
            fr5_B_end = fr5_B_end_from_SDK[1]
            print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz
            fr5_B_end[2] = 190.0
            fr5_B_end[3] = 90.0
            fr5_B_end[4] = 0.0
            fr5_B_end[5] = 0.0

            # fr5_B_end[0] = fr5_B_end[0] + 5.0
            # 计算该期望末端位置逆运动学解算为关节角度joint3
            fr5_B_joint3 = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
            while( type(fr5_B_joint3) != tuple):
                fr5_B_joint3 = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                time.sleep(0.008)
            fr5_B_joint3 = fr5_B_joint3[1]

            # 添加误差
            fr5_B_joint1 = [fr5_B_joint3[i] + noise[i] for i in range(6)]
            fr5_B_joint1 = [float(x) for x in fr5_B_joint1]
            # 正解成pos
            pos_out = fr5_B.robot.GetForwardKin(fr5_B_joint1)
            while( type(pos_out) != tuple):
                pos_out = fr5_B.robot.GetForwardKin(fr5_B_joint1)
                time.sleep(0.008)
            pos_out = pos_out[1]
            print('不准的pos',pos_out)
            print('noise',noise)
            pos_out[0] = pos_out[0] + delta
            # 计算该期望末端位置逆运动学解算为关节角度joint2
            fr5_B_joint2 = fr5_B.robot.GetInverseKin(0,pos_out,-1)
            while( type(fr5_B_joint2) != tuple):
                fr5_B_joint2 = fr5_B.robot.GetInverseKin(0,pos_out,-1)
                time.sleep(0.008)
            fr5_B_joint2 = fr5_B_joint2[1]
            print('joint2=================',fr5_B_joint2)
            print('joint1=================',fr5_B_joint1)
            print('joint3=================',fr5_B_joint3)
            joint_out = [0.,0.,0.,0.,0.,0.]
            for i in range(6):
                joint_out[i] = fr5_B_joint2[i] - fr5_B_joint1[i] + fr5_B_joint3[i]
                # print('di {} ge chazhi'.format(i))
                # print(fr5_B_joint2[i] - fr5_B_joint1[i])
            print('joint_out',joint_out)
            # 执行servoJ
            # fr5_B.robot.ServoJ(fr5_B_joint2,0.0,0.0,0.016,0.0,0.0)
            fr5_B.robot.MoveJ(fr5_B_joint2,0,0)
            
            current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
            current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
            current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
            time.sleep(0.016)

        # 如果x基本对齐了，但是y还没对齐，这个时候只需要慢慢在y轴上移动就行了
        if( (fr5_B_end[0] - current_xyz[0] < 8.0 and (fr5_B_end[0] - current_xyz[0] > -8.0)) and ((fr5_B_end[1] - current_xyz[1] - b_bias > 10.0)or (fr5_B_end[1] - current_xyz[1] - b_bias < -10.0) ) ):
            print('执行y负方向不断跟进')
            # current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
            # current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
            # current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
            # print("current_xyz[1]:",current_xyz[1])
            # print("current_xyz[0]:",current_xyz[0])
            delta = abs(fr5_B_end[1] - current_xyz[1] - b_bias)+5.0

            fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
            while( type(fr5_B_end_from_SDK) != tuple):
                fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
                time.sleep(0.008)
            # print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
            fr5_B_end = fr5_B_end_from_SDK[1]
            print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz
            fr5_B_end[2] = 190.0
            fr5_B_end[3] = 90.0
            fr5_B_end[4] = 0.0
            fr5_B_end[5] = 0.0
            # 计算该期望末端位置逆运动学解算为关节角度joint3
            fr5_B_joint3 = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
            while( type(fr5_B_joint3) != tuple):
                fr5_B_joint3 = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
                time.sleep(0.008)
            fr5_B_joint3 = fr5_B_joint3[1]

            # 添加误差
            fr5_B_joint1 = [fr5_B_joint3[i] + noise[i] for i in range(6)]
            fr5_B_joint1 = [float(x) for x in fr5_B_joint1]
            # 正解成pos
            pos_out = fr5_B.robot.GetForwardKin(fr5_B_joint1)
            while( type(pos_out) != tuple):
                pos_out = fr5_B.robot.GetForwardKin(fr5_B_joint1)
                time.sleep(0.008)
            pos_out = pos_out[1]
            print('不准的pos',pos_out)
            print('noise',noise)
            pos_out[1] = pos_out[1] - delta
            # 计算该期望末端位置逆运动学解算为关节角度joint2
            fr5_B_joint2 = fr5_B.robot.GetInverseKin(0,pos_out,-1)
            while( type(fr5_B_joint2) != tuple):
                fr5_B_joint2 = fr5_B.robot.GetInverseKin(0,pos_out,-1)
                time.sleep(0.008)
            fr5_B_joint2 = fr5_B_joint2[1]
            # fr5_B_end[1] = fr5_B_end[1] - delta
            # # 计算该期望末端位置逆运动学解算为关节角度
            # fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
            # while( type(fr5_B_jointpos_new) != tuple):
            #     fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)
            #     time.sleep(0.008)
            # fr5_B_jointpos_new = fr5_B_jointpos_new[1]
            
            # 执行servoJ
            # fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
            fr5_B.robot.MoveJ(fr5_B_joint2,0,0)
            
            current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
            current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
            current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
            time.sleep(0.008)

        # 如果都对齐了，判断结束条件
        # fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
        # while( type(fr5_B_end) != tuple):
        #     fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
        #     time.sleep(0.5)
        # fr5_B_end = fr5_B_end[1]
        # current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
        # current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
        # current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
        fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
        while( type(fr5_B_end_from_SDK) != tuple):
            fr5_B_end_from_SDK = fr5_B.robot.GetActualToolFlangePose(0)
            time.sleep(0.008)
        # print("fr5_B_end from SDK",fr5_B_end_from_SDK)# 得到机械臂末端xyz，rxryrz
        fr5_B_end = fr5_B_end_from_SDK[1]
        if ( ((fr5_B_end[0] - current_xyz[0] < 10.0) and (fr5_B_end[0] - current_xyz[0] > -10.0)) and (fr5_B_end[1] - current_xyz[1] - b_bias < 15.0)and (fr5_B_end[1] - current_xyz[1] - b_bias > -15.0)      ):
            print("x,y对齐完成")
            fr5_B.robot.MoveGripper(1, 0, 50, 10, 10000, 1) # 夹爪闭合
            time.sleep(3)
            fr5_B.MoveL(0.0,0.0,50.0,40.0)
            time.sleep(2)
            fr5_B.MoveL(0.0,0.0,-50.0,40.0)
            time.sleep(2)
            fr5_B.robot.MoveGripper(1, 100, 50, 10, 10000, 1) # 夹爪张开
            time.sleep(3)
            fr5_B.Go_to_start_zone()
            exit()
    else:
        print('error')


def testik():
    global fr5_A
    global fr5_B
    fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
    fr5_B_end = fr5_B_end[-6:]# 得到机械臂末端xyz，rxryrz
    # print(fr5_B_end)
    # ret = fr5_A.robot.GetInverseKin(0,[-50.0,-400.0,300.0,90.0,0.0,0.0],-1)
    ret = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
    print("ret:",ret)
    # ret = fr5_A.robot.GetInverseKin(0,[-100.0,-400.0,300.0,90.0,0.0,0.0],[-84.16123962402342, -80.7188720703125, -124.6822357177734, -154.598892211914, -84.16123962402342, 5.769830041535158e-15])
    # print("ret:",ret)
    # judge = fr5_A.robot.GetInverseKinHasSolution(0,[-100.0,-400.0,300.0,90.0,0.0,0.0],[-84.16123962402342, -80.7188720703125, -124.6822357177734, -154.598892211914, -84.16123962402342, 5.769830041535158e-15])
    # print("judge:",judge)
    ret = ret[-6:]
    rett = fr5_B.robot.GetForwardKin(ret)
    rett = rett[-6:]
    print("rett:",rett)
    fr5_B.robot.ServoMoveStart()
    time.sleep(0.5)
    count = 200
    input('------按下回车以开始servo------')
    
    while(count):
        count = count -0.25
        # 更新下一个期望末端位置
        rett[0] = rett[0] + 1.0
        # 计算该期望末端位置逆运动学解算为关节角度
        ret = fr5_B.robot.GetInverseKin(0,rett,-1)
        ret = ret[-6:]
        # 通过servoj执行关节角度
        fr5_B.robot.ServoJ(ret,0.0,0.0,0.008,0.0,0.0)
        ret[0] = ret[0] + 0.25
        time.sleep(0.008)

if __name__ == "__main__":
    
    print("---------------FR5机械臂化学协作实验------------------\n") 
    init()
    # 加载模型
    model = joblib.load('/home/wangzy/FR5_exp/fr5_gpt/src/scripts/fr5init/model.pkl')
    
    vision_arm_test.talker()
    # testik()
    # input('------按下回车以开始抓取烧杯------')

    fr5_B.robot.MoveCart([vision_arm_test.cup_loc_to_base[0],vision_arm_test.cup_loc_to_base[1] + b_bias + 100.0,190.0,90.0,0.0,0.0], 0, 0)
    print(vision_arm_test.cup_loc_to_base[0],vision_arm_test.cup_loc_to_base[1])
    # time.sleep(2)
    noise = add_noise(range1=2,gaussian=True)
    
    input('------按下回车以开始抓取烧杯------')

    while True:
        # input('------按下回车以开始抓取烧杯------')
        # print(vision_arm_test.cup_loc_to_base)

        vision_catch_cup(fr5_B,noise)

        # 线性定位误差补偿
        # vision_arm_test.cup_loc_to_base[0] = vision_arm_test.cup_loc_to_base[0]*0.9

        
        # print('杯子相对于基坐标系的位置：',vision_arm_test.cup_loc_to_base)
        # time.sleep(1)


        # print(vision_arm_test.cup_loc_to_base)

    # try:
    #     talker()
    # except rospy.ROSInterruptException as e:
    #     print(e,"\n")