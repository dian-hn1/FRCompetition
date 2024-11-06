#!/bin/python3
# -*- coding: utf-8 -*-
import rospy
import time
from math import sin, cos,pi
import numpy as np
from std_msgs.msg import Float32MultiArray,String
import tf
import tf_eyeoffhand
import joblib
# from chemistry import ChemistryTest
# from chemistry import main


# 在机械臂A坐标系下，相机坐标系原点的坐标
camera_center_to_A = [-30.0,-832.0,890.0]
# 在机械臂A坐标系下，tag中心的坐标
tag_center_to_A = [0.0,0.0]
# 在机械臂A坐标系下，物体中心的坐标
obj_center_to_A = [0.0,0.0]
# 相机坐标系下，tag中心的坐标
tag_center_to_camera = []

tag_trans_mat = []

tag_to_obj = 105.0
cup_position = [[],[],[]]
cup_loc_to_base = []
catcher_pos = [0.0,0.0,0.0]

# camera2base = [ 
#  [ 0.99885901,  0.02414404  ,0.04120379,99.50256158],
#  [ 0.02214817, -0.99859083 , 0.04822672,-542.01673407],
#  [ 0.04231012 ,-0.0472591 , -0.99798619,846.15889269],
#  [ 0.0,0.0,0.0,1.0]
# ]

# camera2base =[[0.9997805823477901, 0.020127193023417297, 0.005803728225238367, 99.571550487483], 
#               [0.019472684607192176, -0.9951305611617784, 0.09662287925762053, -546.9657789771323], 
#               [0.0077202146669083805, -0.0964886643230281, -0.9953041424321789, 845.3018289960025], 
#               [0.0, 0.0, 0.0, 1.0]]
camera2base = [[0.9985750339347319, 0.022189620209739077, 0.048533723916429926, 77.91185167719914], 
               [0.019898166421089946, -0.9986873577260162, 0.047197717013712184, -500.246214051216], 
               [0.049517315914004385, -0.046164729753085165, -0.9977057948871988, 840.3848447109431], 
               [0.0, 0.0, 0.0, 1.0]]# yuan lai shi -538

# camera2base = [[0.9989815796956489, -0.008677174836281193, -0.044277647471905573, 73.47391305150836], 
#                [-0.005560729886305628, -0.9975295329368643, 0.07002791730369327, -536.0244091069762], 
#                [-0.04477590548405788, -0.06971038341324744, -0.9965618800317724, 837.7780418490264], 
#                [0.0, 0.0, 0.0, 1.0]]
# 加载模型
model = joblib.load('/home/wangzy/FR5_exp/fr5_gpt/src/scripts/fr5init/model.pkl')

def tf_tag_left_to_A(tag_center_to_camera):
    '''
    tag放在物体左侧时（即当且仅当tag的y坐标比物体y坐标少tag_to_obj，x坐标一致时）
    '''
    global tag_center_to_A
    global obj_center_to_A
    if tag_center_to_camera == []:
        pass
    else :
        # print(tag_center_to_camera)
        tag_center_to_camera = tag_center_to_camera.split( )
        tag_center_to_camera = [float(num) for num in tag_center_to_camera]
        # 坐标转换
        tag_center_to_A[0] = (tag_center_to_camera[0] - camera_center_to_A[0])*(-1.0)
        tag_center_to_A[1] = tag_center_to_camera[1] + camera_center_to_A[1]
        obj_center_to_A[0] = (tag_center_to_camera[0] - camera_center_to_A[0])*(-1.0)
        obj_center_to_A[1] = tag_center_to_camera[1] + camera_center_to_A[1] - tag_to_obj
        # 结果保留三位小数
        for i in range(2):
            tag_center_to_A[i] = f'{tag_center_to_A[i]:.3f}'
            obj_center_to_A[i] = f'{obj_center_to_A[i]:.3f}'

def camera_callback(rece_tag_center_to_camera):
    '''
    回调函数
    '''
    global tag_center_to_camera
    # 前几帧接收是空白，舍弃
    if rece_tag_center_to_camera.data == []:
        pass
    else :
        tag_center_to_camera = rece_tag_center_to_camera.data

def camera_callback2(rece_tag_trans_mat):
    '''
    回调函数，得到tag_to_camera的变换矩阵
    '''
    global tag_trans_mat
    global cup_loc_to_base
    global model
    if rece_tag_trans_mat.data == []:
        print('?')
        pass
    else :
        # print('++++')
        tag_trans_mat = rece_tag_trans_mat.data
        tag_trans_mat = list(tag_trans_mat)
        tag_trans_mat = [tag_trans_mat[i:i+4] for i in range(0, len(tag_trans_mat), 4)]
        # print(tag_trans_mat,'\n')
        M1 = tag_trans_mat
        M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
        cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
        # cup_loc_to_base[0] = cup_loc_to_base[0]-40.0
        # if cup_loc_to_base[0] > 0:
        #     cup_loc_to_base[0] = cup_loc_to_base[0] + 30.0
        # else:
        #     cup_loc_to_base[0] = cup_loc_to_base[0] - 30.0
        # cup_loc_to_base[1] = cup_loc_to_base[1] + 20.0
        # 线性回归矫正
        cup_loc_correct = model.predict([[cup_loc_to_base[0],cup_loc_to_base[1]]])
        cup_loc_to_base[0] = cup_loc_correct[0][0]-45.0
        cup_loc_to_base[1] = cup_loc_correct[0][1]-35.0

        

# 杯子位置回调函数
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
        cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
        # cup_loc_to_base[0] = cup_loc_to_base[0]-40.0
        # cup_loc_to_base[1] = cup_loc_to_base[1]-30.0
        # 线性回归矫正
        cup_loc_correct = model.predict([[cup_loc_to_base[0],cup_loc_to_base[1]]])
        cup_loc_to_base[0] = cup_loc_correct[0][0]-45.0
        cup_loc_to_base[1] = cup_loc_correct[0][1]-35.0

        # print(cup_loc_to_base)# 物体在camera下的坐标，旋转矩阵，，，默认正放
    

def talker():
    # rospy.init_node('vision_arm_test', anonymous=True)
    # rospy.Subscriber('/tag_pose',String,camera_callback)
    rospy.Subscriber('/tag_trans_mat',Float32MultiArray,camera_callback2)
    rospy.Subscriber('/cup_loc',Float32MultiArray,cup_loc_callback)
    # rospy.Subscriber('/tag_trans_mat',Float32MultiArray,cup_loc_callback)
    # rospy.Subscriber('/cup_loc',Float32MultiArray,camera_callback2)
    
    while not rospy.is_shutdown():
        pass
    # print('l')
        # tf_tag_to_A(tag_center_to_camera)
        
# if __name__ == "__main__":
#     try:
#         # 加载模型
#         # model = joblib.load('model.pkl')
#         talker()
#     except rospy.ROSInterruptException as e:
#         print(e,"\n")
talker()