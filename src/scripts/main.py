#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 这是使用法奥api进行路径规划的demo，可以直接运行

import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
import rospy
import time
from math import sin, cos, pi
from chemistryexp import HNchemistry
import struct
import copy

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
solid_beaker_pos = [0, 0]
liquid_beaker_pos0 = [75, 185]
liquid_beaker_pos1 = [0, -600]
funnel_pos = [200, -800]
dish_pos = [-500, -700]

# 初始化化学实验对象
def init():
    global fr5_A
    global fr5_B
    fr5_A = HNchemistry(1)
    # fr5_B = HNchemistry(2)
    # fr5_A.dou_go_start(fr5_B)
    fr5_A.Go_to_start_zone()
        
# def coordinate_transform(A_cordiate):                         # 双臂坐标系转换方法
#     B_cordiate = [ - A_cordiate[0], - 1300 - A_cordiate[1]]   # 1200是两机械臂基座的距离，待定
#     return B_cordiate
def coordinate_tranform(coordinate):
    return [coordinate[0] + 200, - coordinate[1] - 600]

if __name__ == "__main__":
    ############################# 开始 ###############################
    print("---------------FR5机械臂化学协作实验------------------\n") 
    init()
    fr5_A.pick(coordinate_tranform(solid_beaker_pos), "yn", 2)
    fr5_A.pour(50, 50, coordinate_tranform(liquid_beaker_pos0), "yn", 2, upright=0, max_angel=100.0, shake=1)
    fr5_A.put(coordinate_tranform(solid_beaker_pos), "yn", 2)
    
    # # 待完成：搅拌溶解
    
    # fr5_B.pick(coordinate_transform(funnel_pos), "yn", 6)
    # fr5_B.Safe_move(coordinate_transform(liquid_beaker_pos1) + [225], "yn")    # 增加是漏斗悬空的高度
    # fr5_A.pick(copy.deepcopy(liquid_beaker_pos0), "xp", 2)
    # fr5_A.pour(5, 20, copy.deepcopy(liquid_beaker_pos1), "yn", 6, max_angel=110.0, shake=0)
    # fr5_A.put(copy.deepcopy(liquid_beaker_pos0), "xp", 2)
    # time.sleep(5)                                                        # 等待过滤完成
    # fr5_B.put(coordinate_transform(funnel_pos), "yn", 6)
    # fr5_A.pick(copy.deepcopy(liquid_beaker_pos1), "yn", 2)
    # fr5_A.pour(5, 20, copy.deepcopy(dish_pos), "yn", 7, max_angel=110.0, shake=0)
    # fr5_A.put(copy.deepcopy(liquid_beaker_pos1), "yn", 2)
    
    
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