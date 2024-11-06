import os
import sys
# 获取parent_folder文件夹的路径
parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# 将parent_folder文件夹添加到sys.path列表中
sys.path.append(parent_path)
sys.path.append("../")
# from fr5_init_new import fr5robot
from chemistryexp import HNchemistry
import re
import time
from loguru import logger
# import openai
# global fr5
# global fr5_A
global fr5_B
# fr5_A = HNchemistry(1)      #192.168.59.6（左侧机械臂）
fr5_B = HNchemistry(2)      #192.168.58.6(右侧机械臂)



if __name__ == '__main__':

    # fr5_A.dou_go_start(fr5_B)
    water = [600, -100]
    crude_salt = [600, 0]
    funnel = [-500,-500]
    cup_guolv = [-500,-400]
    operate_area = [200,-700]
    test = [600,0]
    
    fr5_B.Go_to_start_zone(open=0)
    # fr5_B.MoveL(0,-100,0)
    # fr5_B.MoveL(0,-100,0)
    # fr5_B.MoveL(0,-7,0)
    # fr5_B.MoveL(0,0,-100)
    # fr5_B.MoveL(0,0,-100)
    # fr5_B.MoveL(0,0,-100)
    
    # fr5_B.MoveGripper(1, 0, 50, 10, 10000, 1) # close
    # fr5_B.MoveL(0,0,100)
    # fr5_B.MoveL(100,0,0)
    # fr5_B.MoveL(0,0,-100)
    # fr5_B.MoveGripper(1, 100, 50, 10, 10000, 1) # open
    # fr5_B.MoveGripper(1, 100, 50, 10, 10000, 1)
    # fr5_B.MoveL(0,-100,0)
    # fr5_B.MoveL(0,-100,0)
    # fr5_B.MoveL(100,0,0)
    # fr5_B.MoveL(100,0,0)
    # fr5_B.MoveL(100,0,0)
    # fr5_B.pick(test,"yn" , "3" , False, is_display = False)
    # fr5_B.Go_to_start_zone(open=0)
    
    # fr5_A.MoveL(0,-50,-20)
    # fr5_A.MoveL(0,-50,-20)
    # fr5_A.MoveL(0,0,-20)
    # time.sleep(1)
    # fr5_A.MoveL(-50,0,0)
    # time.sleep(1)
    # fr5_A.MoveL(0,-100,0)
    # time.sleep(1)
    # fr5_A.MoveL(100,0,0)
    # time.sleep(1)
    # fr5_A.MoveL(0,100,0)
    # time.sleep(1)
    # fr5_A.MoveL(-50,0,0)
    
    