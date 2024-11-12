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
# import vision_arm_test
import tf_eyeoffhand
import joblib
from std_msgs.msg import Float32MultiArray,String

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
camera2base = [[0.9985750339347319, 0.022189620209739077, 0.048533723916429926, 77.91185167719914], 
        [0.019898166421089946, -0.9986873577260162, 0.047197717013712184, -538.246214051216], 
        [0.049517315914004385, -0.046164729753085165, -0.9977057948871988, 840.3848447109431], 
        [0.0, 0.0, 0.0, 1.0]]
model = joblib.load('/home/wangzy/FR5_exp/fr5_gpt/src/scripts/fr5init/model.pkl')
global cup_position
cup_position = [[],[],[]]
global cup_loc_to_base
global tag_trans_mat
global tag_loc_to_base
tag_loc_to_base = [[],[],[]]
cup_loc_to_base = [[],[]]



def add_noise(range1, gaussian=False):
        noiselist = []
        for i in range(6):
                if gaussian:
                        angle = np.clip(np.random.normal(0, 1) * range1, -range1, range1)
                else:
                        angle += random.uniform(-5, 5)
                noiselist.append(angle)
        return noiselist

def cup_loc_callback(data):
        global cup_position
        global cup_loc_to_base
        cup_loc = data.data
        # print('--',len(cup_loc))
        if len(cup_loc) > 2:
                # cup_position[0] = (cup_loc[1])/1000
                # cup_position[1] = (cup_loc[0]-3)/1000
                # print('3333')
                cup_position[0] = [cup_loc[0]]
                cup_position[1] = [cup_loc[1]]
                cup_position[2] = [cup_loc[2]]
                # 进行杯子到基坐标系的解算
                translate_list = cup_position
                rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
                M1 = tf_eyeoffhand.get_transform_mat_from_RT(rotation_list,translate_list)
                # M1 = tag_trans_mat
                M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
                cup_rot, tmp = tf_eyeoffhand.get_RT_from_transform_mat(M2)
                # 线性回归矫正
                cup_loc_correct = model.predict([[tmp[0],tmp[1]]])
                cup_loc_to_base[0] = cup_loc_correct[0][0]-45.0
                cup_loc_to_base[1] = cup_loc_correct[0][1]-35.0
                # print("cup_loc_correct:",cup_loc_to_base)

def tag_callback(rece_tag_trans_mat):
        '''
        回调函数，得到tag_to_camera的变换矩阵
        '''
        global tag_trans_mat
        global tag_loc_to_base
        global model
        if rece_tag_trans_mat.data == []:
                print('Empty return')
                pass
        else :
                # print('++++')
                tag_trans_mat = rece_tag_trans_mat.data
                tag_trans_mat = list(tag_trans_mat)
                tag_trans_mat = [tag_trans_mat[i:i+4] for i in range(0, len(tag_trans_mat), 4)]
                # print(tag_trans_mat,'\n')
                M1 = tag_trans_mat
                M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
                cup_rot, tmp = tf_eyeoffhand.get_RT_from_transform_mat(M2)
                # 线性回归矫正
                tag_loc_correct = model.predict([[tmp[0],tmp[1]]])
                tag_loc_to_base[0] = tag_loc_correct[0][0]-45.0
                tag_loc_to_base[1] = tag_loc_correct[0][1]-35.0
                tag_loc_to_base[2] = tmp[2]
                # print("tag_loc_to_base:",tag_loc_to_base)


# use new sdk
if __name__ == '__main__':
        # init fr5B
        # rospy.init_node('fr5_b')
        # rospy.Subscriber('/tag_pose',String,camera_callback)
        rospy.Subscriber('/tag_trans_mat',Float32MultiArray,tag_callback)
        rospy.Subscriber('/cup_loc',Float32MultiArray,cup_loc_callback)
        # while not rospy.is_shutdown():
        #         pass
        
        ############ init robot
        fr5_B = fr5robot(2)
        fr5_B.Go_to_start_zone()
        # fr5_B.MoveL(y=-50,z=-100)
        fr5_B.MoveL(y=-100,z=-100)
        z_bar = 200

        while True: 
                x_bar = tag_loc_to_base[0]-cup_loc_to_base[0]
                y_bar = tag_loc_to_base[1]-cup_loc_to_base[1]
                # print(tag_loc_to_base[2])
                # for _ in range(20):
                if (abs(y_bar)<110 and abs(x_bar)<=20) :
                        # close to object stop
                        fr5_B.robot.MoveGripper(1, 0, 50, 10, 10000, 1)
                        fr5_B.MoveL(z=100)
                        time.sleep(1)
                        fr5_B.MoveL(z=-100)
                        fr5_B.robot.MoveGripper(1, 100, 50, 10, 10000, 1)
                        fr5_B.MoveL(y=30)
                        fr5_B.Go_to_start_zone()
                        fr5_B.MoveL(y=-50,z=-100)
                        z_bar = 200

                        # break
                # elif tag_loc_to_base[0]>=90: 
                #         fr5_B.MoveL(y=30)
                #         fr5_B.Go_to_start_zone()
                #         fr5_B.MoveL(y=-50,z=-100)
                else:
                        if tag_loc_to_base[2] >= 220:
                                # for i in range(10):
                                ret_right = fr5_B.robot.ServoCart(1, [-x_bar/100, -y_bar/500,-300/70,0,0,0],cmdT = 0.001,vel=4)
                                z_bar = z_bar - (150/400)
                                # print("discount:",z_bar)
                        else:
                                ret_right = fr5_B.robot.ServoCart(1, [-x_bar/20, -y_bar/80,0,0,0,0],cmdT = 0.001,vel=4)
                                # print("moving:",z_bar)
                        time.sleep(0.01)
                

                
        #         pos_now = fr5_B.robot.GetActualToolFlangePose(0)[1]
        #         fr5_B_end = [block1_pos[0], block1_pos[1] , b_bias-50, pos_now[3],pos_now[4],pos_now[5]]
        #         # 计算该期望末端位置逆运动学解算为关节角度
        #         fr5_B.robot.MoveL(fr5_B_end,0,0)
        #         # print("error code:"+ret)
                
        #         input('按下回车来闭合夹爪')
        #         fr5_B.robot.MoveGripper(1, 0, 50, 10, 10000, 1)
        #         time.sleep(3)
        #         fr5_B.MoveL(0.,0.,200.0)
        
                