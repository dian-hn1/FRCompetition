import os
import sys
# 获取parent_folder文件夹的路径
parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# 将parent_folder文件夹添加到sys.path列表中
sys.path.append(parent_path)
sys.path.append("../")
sys.path.append("/home/wangzy/FR5_exp/fr5_gpt/src/scripts/robot_follow")
from chemistryexp import HNchemistry
from unity_robotics_demo_msgs.msg import vr

import time
import Robot
import random
import numpy as np
from scipy.spatial.transform import Rotation as R # 用于旋转矩阵的转换
import math
import rospy
from std_msgs.msg import UInt8MultiArray,Float32MultiArray,String
import struct
import socket


from kalman_filter import kalman_filter,kalman_filter_pose
    
# robot_right = Robot.RPC("192.168.58.6")
robot_right = HNchemistry(2)
robot_left = HNchemistry(1)
# robot_left = Robot.RPC("192.168.59.6")

left_pose = [0.,0.,0.,0.,0.,0.]
right_pose = [0.,0.,0.,0.,0.,0.]
global gripper,gripper_done,arm_run
gripper = True
gripper_done = True
arm_run = False
tag2base = [] 
# -200,-700
# 位置回调函数
def key_click(key):
    global gripper,gripper_done,arm_run
    if key == 'r2':
        # 左臂夹爪
        if gripper == True:
            gripper = False
            gripper_done = False
            robot_right.robot.MoveGripper(1, 0, 100, 1, 10000, 1)
            time.sleep(1)
            gripper_done = True
        else:
            gripper = True
            gripper_done = False
            robot_right.robot.MoveGripper(1, 100, 100, 1, 10000, 1)
            time.sleep(1)
            gripper_done = True
    elif key == 'r1':
        # 暂停和启动
        if arm_run == False:
            print("start!")
            time.sleep(2)
            arm_run = True
        else:
            print("stop")
            arm_run = False
    elif key == 'l2':
        # 右臂夹爪
        if gripper == True:
            gripper = False
            gripper_done = False
            robot_left.robot.MoveGripper(1, 0, 100, 1, 10000, 1)
            time.sleep(1)
            gripper_done = True
        else:
            gripper = True
            gripper_done = False
            robot_left.robot.MoveGripper(1, 100, 100, 1, 10000, 1)
            time.sleep(1)
            gripper_done = True
            
    elif key == 'l1':
        print("划火柴")
        gripper_done = False
        for _ in range(100):
            robot_left.robot.ServoCart(1, [0.5,0,0,0,0,0],cmdT = 0.001,vel=1)
        print("划火柴结束")
        gripper_done = True

def tag_loc_callback(data):
    # 手柄位置
    pos_handle = str(data.device)
    # 按键信息
    key = str(data.order)
    # print(key)
    # 摇杆信息
    rocker = str(data.axis)
    if key:
        key_click(key)
    else:
        pos = pos_handle.replace('Left,',' ').strip(" ")
        pos = pos.replace(';Right','')
        # print(pos)
        # 去掉字符串中的空格，并将其分割成一个列表
        str_values = pos.split(",")
        # print(str_values)
        # 将字符串列表转换为浮点数列表
        pos = [float(value) for value in str_values]
        # print(float_values)

        left_pose[0] = pos[0]
        left_pose[1] = pos[1]
        left_pose[2] = pos[2]
        left_pose[3] = pos[3]
        left_pose[4] = pos[4]
        left_pose[5] = pos[5]
        right_pose[0] = pos[6]
        right_pose[1] = pos[7]
        right_pose[2] = pos[8]
        right_pose[3] = pos[9]
        right_pose[4] = pos[10]
        right_pose[5] = pos[11]

def angle_change(current_angle, previous_angle):
    # 计算基础角度差值
    angle_change = current_angle - previous_angle

    # 处理跳变情况
    if angle_change > 180:
        angle_change -= 360
    elif angle_change < -180:
        angle_change += 360

    return angle_change
    
def get_right_data():
    pos_last = right_pose.copy()
    time.sleep(0.05)
    pos_aim = right_pose.copy()
    
    return np.array([pos_aim[0]-pos_last[0], pos_aim[1]-pos_last[1], pos_aim[2]-pos_last[2],angle_change(pos_aim[3],pos_last[3]), angle_change(pos_aim[4],pos_last[4]), angle_change(pos_aim[5],pos_last[5])])

def get_left_data():
    pos_last = left_pose.copy()
    time.sleep(0.05)
    pos_aim = left_pose.copy()
    return np.array([pos_aim[0]-pos_last[0], pos_aim[1]-pos_last[1], pos_aim[2]-pos_last[2],angle_change(pos_aim[3],pos_last[3]), angle_change(pos_aim[4],pos_last[4]), angle_change(pos_aim[5],pos_last[5])])

def arm_move(s_pos,s_ori,mode=0):
    if mode == 0:
        ret_right = robot_right.robot.ServoCart(1, [-estimates_right[4][2]*s_pos, estimates_right[4][0]*s_pos, estimates_right[4][1]*s_pos,right_new[3]*s_ori,right_new[5]*s_ori,-right_new[4]*s_ori],cmdT = 0.001,vel=1)
        ret_left = robot_left.robot.ServoCart(1, [estimates_left[4][2]*s_pos, -estimates_left[4][0]*s_pos, estimates_left[4][1]*s_pos, left_new[3]*s_ori,left_new[5]*s_ori,-left_new[4]*s_ori],cmdT = 0.001,vel=1)
    # if ret_right != 0 or ret_left != 0:
                        #     print("机械臂错误码：right",ret_right,"left:",ret_left)
                        #     print(estimates_right[4][2])
                        #     print(estimates_right[4][1])
                        #     print(estimates_right[4][0])

if __name__ == "__main__":
    # rospy.init_node("Fr5_vr")
    rospy.Subscriber('vr_control',vr,tag_loc_callback)
    ###############初始化机械臂####################
    print("-----------------------开始复位--------------------------")
    # 初始化机械臂
    robot_right.Go_to_start_zone()
    robot_left.Go_to_start_zone()                    

    
    print("-----------------------开始执行--------------------------")
    
    # input("__________按enter开始执行__________s")
    measurements_right = np.array([get_right_data() for _ in range(5)])
    measurements_left = np.array([get_left_data() for _ in range(5)])
    # print(f"测量值: {measuremsadsadasents}")
    
    while True:
        try:
            # 更新获取到的手柄位置
            right_new = get_right_data()
            left_new = get_left_data()
            # print(pos)
            for i in range(4):
                measurements_right[i] = measurements_right[i+1]
                measurements_left[i] = measurements_left[i+1]
            measurements_right[4] = right_new
            measurements_left[4] = left_new
            
            if arm_run:
                # 卡尔曼滤波，平滑轨迹
                estimates_right = kalman_filter(measurements = measurements_right)
                estimates_right_pose = kalman_filter_pose(measurements = measurements_right)

                estimates_left = kalman_filter(measurements = measurements_left)
                estimates_left_pose = kalman_filter_pose(measurements = measurements_left)
                
                # 机械臂执行动作
                if gripper_done:
                    if all(abs(estimates_right[4][i])<0.05 and abs(estimates_left[4][i])<0.05 for i in range(3)) \
                        and all(abs(left_new[i])< 25 and abs(right_new[i])< 25 for i in range(3,6)):
                        for i in range(25):
                            arm_move(150,0.1)
                            # time.sleep(0.001)
                    else:
                        print("速度太快了")

                        
            pass
        except KeyboardInterrupt:
            print("程序被用户中断")
            pass


