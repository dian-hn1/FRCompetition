#!/bin/python3
# -*- coding: utf-8 -*-
import rospy
import time
from math import sin, cos,pi
import numpy as np
from std_msgs.msg import Float32MultiArray,String
import vision_arm_test
import tf_eyeoffhand
from chemistry import ChemistryTest
import frrpc
import chemistry
import joblib

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
# camera2base =[[0.9989000103437977, -0.0015769329107144041, -0.04686451341639644, 105.03969030113437], 
#  [0.0037459396993370048, -0.9935565796525974, 0.11327528841190708, -552.6452898762774],
#   [-0.04674117318734551, -0.11332623840764726, -0.9924577705965345, 866.362433625427], 
#   [0.0, 0.0, 0.0, 1.0]]
fr5_A = []
fr5_B = []
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
    fr5_A = ChemistryTest(1)
    fr5_B = ChemistryTest(2)
    time.sleep(0.5)
    fr5_A.dou_go_start(fr5_B)
    fr5_A.robot.ServoMoveStart()

def vision_catch_cup(fr5_B):
    global catch_xy
    if vision_arm_test.cup_position == [[],[],[]]:
        # print('sdsdsd')
        pass
    else :
        # translate_list = vision_arm_test.cup_position
        # rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
        # M1 = tf_eyeoffhand.get_transform_mat_from_RT(rotation_list,translate_list)
        # M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
        # cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
        # catch_xy = [cup_loc_to_base[0],cup_loc_to_base[1]]
        # catch_xy = ' '.join([str(num) for num in catch_xy])  
        # print('catch_xy:',catch_xy)
        vision_servo(vision_arm_test.cup_loc_to_base)
        time.sleep(0.001)
        # fr5_B.F101_catch_02(catch_xy,"ym","2","1",False)# without sensor

def vision_servo(catch_xyz):
    '''
    视觉伺服部分代码
    '''
    global previous_xyz
    global current_xyz
    global update_flag
    global update_flagg
    global cnt4servo
    current_time = time.time() # 获取当前时间
    # previous_xyz = catch_xyz# 当前帧的xyz坐标
    current_xyz = [0,0,0]
    if (update_flag == 1 and update_flagg == 1):
        previous_xyz = catch_xyz# 当前帧的xyz坐标

    # 每9帧更新一次
    while(cnt4servo %9 == 0):
        cnt4servo = 71
        # translate_list = vision_arm_test.cup_position
        # rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
        # M1 = tf_eyeoffhand.get_transform_mat_from_RT(rotation_list,translate_list)
        # M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
        # cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
        # print(cup_loc_to_base[0],cup_loc_to_base[1],cup_loc_to_base[2])
        current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
        current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
        current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
        # current_xyz = cup_loc_to_base.item()# 下一帧的xyz坐标
        # for i in range(3):
        #     current_xyz[i] = float(cup_loc_to_base[i])
        # print(type(current_xyz))
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
        # translate_list = vision_arm_test.cup_position
        # rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
        # M1 = tf_eyeoffhand.get_transform_mat_from_RT(rotation_list,translate_list)
        # M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
        # cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
        # print(cup_loc_to_base[0],cup_loc_to_base[1],cup_loc_to_base[2])
        current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
        current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
        current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
        # current_xyz = float(current_xyz)
        # for i in range(3):
        #     current_xyz[i] = f'{current_xyz[i]:.3f}'
        if all(abs(delta) < change_threshold for delta in delta_xyz):# 如果xyz坐标的变化量小于阈值，认为目标物体稳定
            print("The target object is stable. Moving the robotic arm closer.")
            
            # 在这里编写靠近目标的代码

            # 先将夹爪xy移动到目标x，距离y100的位置，记得考虑sensor有无

            # 将末端z与目标z对齐，先按照桌面高度测试，后续可以根据实际情况调整

            # 开始伺服，缓慢向目标移动，计算当前夹爪与目标的差值，deltax与deltay，然后根据差值进行移动
            # print(type(current_xyz))
            fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
            print("fr5_B_end:",fr5_B_end)
            fr5_B_end = fr5_B_end[-6:]# 得到机械臂末端xyz，rxryrz
            print(fr5_B_end)
            # fr5_A.robot.ServoCart(0,[-current_xyz[0],fr5_B_end[1],fr5_B_end[2],90.0,0.0,0.0] ,[1.0,1.0,1.0,1.0,1.0,1.0], 0.0, 0.0, 0.008, 0.0, 0.0)
            # fr5_A.robot.MoveCart([-current_xyz[0],fr5_B_end[1],fr5_B_end[2],90.0,0.0,0.0] ,0, 0, 0.0, 0.0, 20.0, -1.0, -1)
            # 对齐y，缓慢向前一小段距离，每次移动1mm，直到deltay小于某一阈值
            print("==================fr5_B_end[1]:",fr5_B_end[1])
            print("current_xyz[1]:",current_xyz[1])
            print("fr5_B_end[1] - chemistry.a_bias:",fr5_B_end[1] - chemistry.b_bias)

            # 对齐x
            x_flag = 1
            y_flag = 1
            while(x_flag):
                if fr5_B_end[0] - current_xyz[0] > 2.0:

                    print('执行x负方向不断跟进')
                    print("current_xyz[0]:",current_xyz[0])
                    # fr5_B_jointpos_old = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
                    # print("ret:",fr5_B_jointpos_old)# 得到当前关节角度
                    fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)[1:7]
                    print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz
                    fr5_B_end[0] = fr5_B_end[0] - 1.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)[1:7]
                    # 执行servoJ
                    fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
                    current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
                    current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
                    current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
                    time.sleep(0.008)
                elif fr5_B_end[0] - current_xyz[0] < -2.0:
                    print('执行x正方向不断跟进')
                    print("current_xyz[0]:",current_xyz[0])
                    # fr5_B_jointpos_old = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
                    # print("ret:",fr5_B_jointpos_old)
                    fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)[1:7]
                    print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz
                    fr5_B_end[0] = fr5_B_end[0] + 1.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)[1:7]
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

            # 对齐y
            while(y_flag):
                if (fr5_B_end[1] - chemistry.b_bias - current_xyz[1]) > 2.0:
                    # translate_list = vision_arm_test.cup_position
                    # rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
                    # M1 = tf_eyeoffhand.get_transform_mat_from_RT(rotation_list,translate_list)
                    # M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
                    # cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
                    # print(cup_loc_to_base[0],cup_loc_to_base[1],cup_loc_to_base[2])
                    # current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
                    # current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
                    # current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])

                    if((fr5_B_end[0] - current_xyz[0])>2.0 or (fr5_B_end[0] - current_xyz[0])<-2.0):
                        y_flag = 0
                        x_flag = 1
                    # fr5_A.robot.MoveCart([current_xyz[0],current_xyz[1],current_xyz[2],90.0,0.0,0.0] ,0, 0, 0.0, 0.0, 50.0, -1.0, -1)
                    # fr5_A.MoveL(0.0,-1.0,0.0,30.0)
                    print('执行y负方向不断跟进')
                    print("current_xyz[1]:",current_xyz[1])
                    print("current_xyz[0]:",current_xyz[0])
                    # fr5_A.robot.ServoCart(1,[0.0,3.0,0.0,0.0,0.0,0.0] ,[0.0,1.0,0.0,0.0,0.0,0.0], 0.0, 0.0, 0.008, 0.0, 0.0)
                    # 不能用servocart，只能用servoj,servocart狗都不用啊
                    # 根据轨迹，给出6dof，再换算为期望关节角度，再去servoj执行
                    # fr5_B_jointpos_old = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
                    # print("ret:",fr5_B_jointpos_old)# 得到当前关节角度
                    fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)[1:7]
                    print("fr5_B_end",fr5_B_end)# 得到机械臂末端xyz，rxryrz
                    fr5_B_end[1] = fr5_B_end[1] - 1.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)[1:7]
                    # 执行servoJ
                    fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
                    time.sleep(0.008)

                elif (fr5_B_end[1] - chemistry.b_bias - current_xyz[1]) < -2.0:

                    # translate_list = vision_arm_test.cup_position
                    # rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
                    # M1 = tf_eyeoffhand.get_transform_mat_from_RT(rotation_list,translate_list)
                    # M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
                    # cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
                    # print(cup_loc_to_base[0],cup_loc_to_base[1],cup_loc_to_base[2])
                    current_xyz[0] = float(vision_arm_test.cup_loc_to_base[0])
                    current_xyz[1] = float(vision_arm_test.cup_loc_to_base[1])
                    current_xyz[2] = float(vision_arm_test.cup_loc_to_base[2])
                    
                    if((fr5_B_end[0] - current_xyz[0])>3.0 or (fr5_B_end[0] - current_xyz[0])<-2.0):
                        y_flag = 0
                        x_flag = 1
                        # print('y_flag',y_flag)
                    print('执行y正方向不断跟进')
                    print("current_xyz[1]:",current_xyz[1])
                    # fr5_B_jointpos_old = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
                    # print("ret:",fr5_B_jointpos_old)
                    fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)[1:7]
                    fr5_B_end[1] = fr5_B_end[1] + 1.0
                    # 计算该期望末端位置逆运动学解算为关节角度
                    fr5_B_jointpos_new = fr5_B.robot.GetInverseKin(0,fr5_B_end,-1)[1:7]
                    # 执行servoJ
                    fr5_B.robot.ServoJ(fr5_B_jointpos_new,0.0,0.0,0.008,0.0,0.0)
                    time.sleep(0.008)

                else:
                    y_flag = 0



        else:# 如果xyz坐标的变化量大于阈值，认为目标物体移动
            print("The target object has moved. Aligning the robotic arm gripper.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            
            # 在这里编写朝向目标物体的代码

            # 先假设z已经对齐

            # 不断对齐x
            # fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
            # fr5_B_end = fr5_B_end[-6:]# 得到机械臂末端xyz，rxryrz
            # # 对齐x，缓慢向前一小段距离，每次移动1mm，直到deltax小于某一阈值
            # if (fr5_B_end[0] ) > current_xyz[0]:
            #     # fr5_A.robot.MoveCart([current_xyz[0],current_xyz[1],current_xyz[2],90.0,0.0,0.0] ,0, 0, 0.0, 0.0, 50.0, -1.0, -1)
            #     fr5_B.MoveL(-1.0,0.0,0.0,30.0)
            # elif fr5_B_end[0] < current_xyz[0]:
            #     # fr5_A.robot.MoveCart([current_xyz[0],current_xyz[1],current_xyz[2],90.0,0.0,0.0] ,0, 0, 0.0, 0.0, 50.0, -1.0, -1)
            #     fr5_B.MoveL(1.0,0.0,0.0,30.0)


def talker():
    # rospy.init_node('vision_arm_test', anonymous=True)
    # rospy.Subscriber('/tag_pose',String,camera_callback)
    pass
    # rospy.Subscriber('/cup_loc',RL_loc_list,cup_loc_callback)
    
    # while not rospy.is_shutdown():
        # pass
    # print('l')
        # tf_tag_to_A(tag_center_to_camera)

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
    model = joblib.load('model.pkl')
    
    vision_arm_test.talker()
    # testik()




    input('------按下回车以开始抓取烧杯------')


    while True:
        # input('------按下回车以开始抓取烧杯------')

        vision_catch_cup(fr5_B)

        # 线性定位误差补偿
        # vision_arm_test.cup_loc_to_base[0] = vision_arm_test.cup_loc_to_base[0]*0.9

        
        # print('杯子相对于基坐标系的位置：',vision_arm_test.cup_loc_to_base)
        # time.sleep(1)


        # print(vision_arm_test.cup_loc_to_base)

    # try:
    #     talker()
    # except rospy.ROSInterruptException as e:
    #     print(e,"\n")