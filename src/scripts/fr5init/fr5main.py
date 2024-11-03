#!/usr/bin/env python
# -*- coding: utf-8 -*-

#这是使用法奥api进行路径规划的demo，可以直接运行
import sys
sys.path.append('/home/zjh/HN-1/demo_ws/src/frcobot_ros/fr5_moveit_config/')

import os
from kyle_robot_toolbox.camera import Camera

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import rospy
import time
from math import sin, cos,pi
from chemistry import ChemistryTest
from chemistry import main
from unity_robotics_demo_msgs.srv import control_order
from scripts import force_sensor
import numpy
import vision_arm_test
import tf_eyeoffhand

fr5_A = []
fr5_B = []
# 相机手眼标定的结果
camera2base = [ 
 [ 0.99885901,  0.02414404  ,0.04120379,99.50256158],
 [ 0.02214817, -0.99859083 , 0.04822672,-542.01673407],
 [ 0.04231012 ,-0.0472591 , -0.99798619,846.15889269],
 [ 0.0,0.0,0.0,1.0]
]
def init():
    global fr5_A
    global fr5_B
    fr5_A = ChemistryTest(1)
    fr5_B = ChemistryTest(2)
    time.sleep(0.5)
    fr5_A.dou_go_start(fr5_B)

def catch(speed,start_catch_position,catch_direction,sel_num,times):
    global fr5_A
    global fr5_B
    fr5_A.robot.SetSpeed(int(speed))
    fr5_A.F101_catch_02(start_catch_position=start_catch_position,catch_direction=catch_direction,sel_num=sel_num,times = times)  # 4种对象抓取

def move(speed,start_catch_position,catch_direction,sel_num,times):
    global fr5_A
    global fr5_B
    fr5_A.robot.SetSpeed(int(speed))
    fr5_A.F101_move_01(start_catch_position=start_catch_position,catch_direction=catch_direction,sel_num=sel_num,times = times)  # 移动
    
def put(speed,start_catch_position,catch_direction,switch,end_catch_position,end_catch_direction,sel_num,times):
    global fr5_A
    global fr5_B
    fr5_A.robot.SetSpeed(int(speed))
    if switch == '1' :# 是预设
        fr5_A.F101_put_01(start_catch_position='0 -700',catch_direction="ym",
                        end_catch_position='0 -700',end_catch_direction="ym",sel_num=sel_num,times = times)  # 放置
    else :
        fr5_A.F101_put_01(start_catch_position=start_catch_position,catch_direction=catch_direction,
                        end_catch_position=end_catch_position,end_catch_direction=end_catch_direction,sel_num=sel_num,times = times)  # 放置
    
def pour(speed,start_catch_position,switch,max_angle,times):
    global fr5_A
    global fr5_B
    fr5_A.robot.SetSpeed(int(speed))
    fr5_B.robot.SetSpeed(int(speed))
    start_catch_position = start_catch_position.split(" ")
    start_catch_position = [float(num) for num in start_catch_position]
    print(int(max_angle))
    for i in range(int(times)):
        if switch == '1' :# 是预设
            fr5_A.F101_pour_03(fr5_B, float(speed),1,0.0,0.0,110)  # 倾倒
        else :
            fr5_A.F101_pour_03(fr5_B, float(speed),0,start_catch_position[0],start_catch_position[1],int(max_angle))

def mix(speed, t, times):
    for i in range(int(times)):
        fr5_A.robot.SetSpeed(int(speed))
        fr5_A.F101_mix_01(float(speed),80.0,int(t))
        # fr5_A.EX_to_mix_no_rA(50.0)
        # input("请按回车键继续！")
        # fr5_A.EX_to_mix_no_rA(50.0)

def gua(speed,times,gua_times):
    fr5_A.robot.SetSpeed(40)
    fr5_B.robot.SetSpeed(40)
    for i in range(int(times)):
        fr5_A.New_Gua(gua_times,fr5_B)

def order_req(req):
    '''
    回调函数，接受Unity传回的指令，进行相应操作
    内容：字符串以逗号连在一起
    '''
    order = req.order
    # 切割指令
    order = order.split(",")
    order = [str(num) for num in order]
    print(order)
    if order[0] == "0" and order[4] != ' ':
        catch(speed= order[1],start_catch_position=order[3],catch_direction=order[4],sel_num=order[5],times = order[2]) 
    elif order[0] == "1" and order[4] != ' ':
        move(speed = order[1],start_catch_position=order[3],catch_direction="ym",sel_num=order[4],times = order[2])  
    
    elif order[0] == "2" and order[4]!= ' ':
        put(speed = order[1],start_catch_position=order[4],catch_direction="ym",switch = order[3],
                        end_catch_position=order[5],end_catch_direction="ym",sel_num=order[6],times = order[2]) 
    elif order[0] == "3" and order[4] != ' ' :
        pour(speed = order[1], start_catch_position = order[4] , switch = order[3], max_angle=order[5],times = order[2] )
    elif order[0] == "4":
        mix(speed = order[1], t = order[3],times = order[2])
    elif order[0] == "5":
        gua(speed = order[1],times = order[2],gua_times = order[3])
    elif order[0] == "6":   # 紧急停止
        fr5_A.Stop()
    elif order[0] == "7":   # 复位
        fr5_A.reset(index = int(order[1]))
    elif order[0] == "8":  # 夹爪开关
        fr5_A.GripperReset(index = int(order[1]),mode=int(order[2]))
    elif order[0] == "9":  # 洗瓶
        fr5_A.dou_pour_wash(fr5_B, catch=1)
    elif order[0] == "10":  # 洗瓶
        fr5_A.Ex(50.0)

    return True

def vision_catch(fr5_B):
    global catch_xy
    # 获取tag坐标
    vision_arm_test.tf_tag_left_to_A(vision_arm_test.tag_center_to_camera)
    catch_xy = vision_arm_test.obj_center_to_A
    # list 转 str , 每个元素以空格隔开
    catch_xy = ' '.join([str(num) for num in catch_xy]) 
    print(catch_xy)
    fr5_A.F101_catch_02(catch_xy,"ym","2","1",True)# with sensor

def vision_catch_tag(fr5_B):
    global catch_xy
    if len(vision_arm_test.cup_position) < 3:
        print('tag_trans_mat error!')
        pass
    else :
        # translate_list = vision_arm_test.cup_position
        # rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
        # M1 = tf_.get_transform_mat_from_RT(rotation_list,translate_list)
        M2 = tf_eyeoffhand.tf_get_obj_to_base(vision_arm_test.tag_trans_mat,camera2base)
        cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
        print('tag位置：',cup_loc_to_base)
        catch_xy = [cup_loc_to_base[0],cup_loc_to_base[1]-90.0]
        catcmodelh_xy = ' '.join([str(num) for num in catch_xy]) 
        fr5_B.F101_catch_02(catch_xy,"ym","2","1",False)# without sensor

def vision_catch_cup(fr5_B):
    global catch_xy
    if vision_arm_test.cup_position == [[],[],[]]:
        pass
    else :
        translate_list = vision_arm_test.cup_position
        rotation_list = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
        M1 = tf_eyeoffhand.get_transform_mat_from_RT(rotation_list,translate_list)
        M2 = tf_eyeoffhand.tf_get_obj_to_base(M1,camera2base)
        cup_rot, cup_loc_to_base = tf_eyeoffhand.get_RT_from_transform_mat(M2)
        print('杯子位置：',cup_loc_to_base)
        catch_xy = [cup_loc_to_base[0],cup_loc_to_base[1]]
        catch_xy = ' '.join([str(num) for num in catch_xy]) 
        fr5_B.F101_catch_02(catch_xy,"ym","2","1",False)# without sensor

def test_sensor():
    while True:
        force_sensor.zero_calibration()
        input('请对末端z轴施加一定力量的同时立马按下回车：')
        force_sensor.read_force_sensor()
        input('请停止施加外力，等待一秒钟后按下回车，进行重新标定：')

def test():
    force_sensor.zero_calibration()


if __name__ == "__main__":
    ########################以下代码不可以被注释！！！###############################
    print("---------------FR5机械臂化学协作实验------------------\n") 
    
    init()
    # input('---')

    #############################################################################

    ##################################以下为来宾展示，机械臂表演代码，需要时取消注释即可，同时把其他模块注释####################
    
    # fr5_A.EX_to_mix_no_rA(50.0)
    #fr5_A.F101_pour_03(fr5_B, 50.0,1,0.0,0.0,110)  # 倾倒
    #input('a重新布置场景，按下回车继续')
    # fr5_A.
    # fr5_A.dou_pour_wash(fr5_B, catch=1)
    ################################################################################################################


    ##################################以下代码为视觉部分代码，需要时取消注释即可，同时把其他模块注释####################
    # rospy.init_node("fr5_main", anonymous = True)
    # vision_arm_test.talker()
    # vision_arm_test.talker()
    # time.sleep(2)
    # while True:
    #     input('------按下回车以开始抓取烧杯------')
    #     vision_catch_cup(fr5_B)
    #############################################################################################################


    ##################################以下代码为数字孪生部分代码，需要时取消注释即可，同时把其他模块注释####################
    # 开启ROS与Unity通信服务 回调函数order_req
    service = rospy.Service('fr5_control', control_order, order_req) 
    while not rospy.is_shutdown():
        pass
    #############################################################################################################







    #以下别删，有时间我再维护

    # input("enter to start")
    # fr5_A.dou_pour_wash(fr5_B, catch=1)

    # # fr5_A.F101_pour_03(fr5_B, 50.0,1,0.0,0.0,110)  # 倾倒
    
    
    # fr5_A.MoveL(0.0,0.0,300.0,20.0)
    # time.sleep(3)
    # fr5_B.MoveL(0.0,-150.0,0.0,50.0)
    # for i in range(10):
    #     fr5_B.MoveL(200.0 , 0.0 , 0.0 , 20.0)
    #     fr5_B.MoveL(0.0 , -200.0 , 0.0 , 20.0)
    #     fr5_B.MoveL(-400.0 , 0.0 , 0.0 , 20.0)
    #     fr5_B.MoveL(0.0 , 200.0 , 0.0 , 20.0)
    #     fr5_B.MoveL(200.0 , 0.0 , 0.0 , 30.0)
    # 开启ROS接受力传感器串口信息
    # force_sensor.talker()
    # test_sensor()

    # while True:
    #     input('------点击以继续执行抓取------')
    #     vision_catch_cup(fr5_B)
        

    # fr5_A.F101_catch_02("0.0 -700.0","ym","2","1")
    # fr5_A.robot.SetSpeed(100)
    # fr5_A.F101_move_01("0.0 -700.0","ym","2","1")
    # fr5_A.F101_put_01("0 -700","ym","-200 -700","ym","3","1")
    # fr5_A.F101_mix_01(50.0,80.0,1)


    
    # fr5_A.F101_pour_03(fr5_B, 50.0,1,0.0,0.0,110)  # 倾倒



    # fr5_A.flow3_5(fr5_B, 50.0)
    # fr5_A.New_Gua("1", fr5_B)

    # fr5_B.mix_ctr(1)
    # input()
    # fr5_B.mix_ctr(0)

    

