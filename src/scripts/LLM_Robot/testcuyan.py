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
global fr5_A
global fr5_B
fr5_A = HNchemistry(1)      #192.168.59.6（左侧机械臂）
fr5_B = HNchemistry(2)      #192.168.58.6(右侧机械臂)


def execute_code(code):

    try:
        exec(code)
    except Exception as e:
        print(f"Error executing code: {e}")

def init():
    global fr5_A
    global fr5_B
    fr5_A = HNchemistry(1)          
    fr5_B = HNchemistry(2)          
    # time.sleep(1)
    fr5_A.dou_go_start(fr5_B)

def pick(object,sel_num):
    '''
    sel_num=1时操作左侧机械臂；为2时操作右侧机械臂沿x正方向抓；为3时操作右侧机械臂沿y-方向抓；为5时操作左机械臂抓漏斗
    '''
    cup = object.copy()
    #sel_num=1时操作左侧机械臂；为2时操作右侧机械臂沿x正方向抓；为3时操作右侧机械臂沿y-方向抓
    if sel_num == 1:
        fr5_A.pick(cup, "yn" , "3" , False, is_display = False)
    if sel_num == 2:
        fr5_B.pick(cup, "xp" , "3" , False, is_display = False)
    if sel_num == 3:
        fr5_B.pick(cup, "yn" , "3" , False, is_display = False)
    if sel_num == 5:
        fr5_A.pick(cup, "yn" , "5" , False, is_display = False)
    
def placeObjectOn(object,left=0,right=0,forward=0,back=0,sel_num=1):
    '''
    selnum = 1:zuo bian ji xie bi fang 
    2: you bian ji xie bi fang 
    3: zuo bian ji xie bi fang lou dou
    '''
    cup = object.copy()
    if sel_num == 1:
        fr5_A.put([cup[0]+left-right,cup[1]-forward+back], "yn" , "3",obj_height = 0)
        #放置后先在y方向上后移夹爪，再上升夹爪，防止触碰到器具
        fr5_A.MoveLDelta(0,50,0)       
        fr5_A.MoveLDelta(0,0,200)
    if sel_num == 2:
        fr5_B.put([cup[0]+left-right,cup[1]-forward+back], "yn" , "3",obj_height = 0)
        #放置后先在y方向上后移夹爪，再上升夹爪，防止触碰到器具
        fr5_B.MoveLDelta(0,50,0)       
        fr5_B.MoveLDelta(0,0,200)
    if sel_num == 3:
        fr5_A.MoveLDelta(0,0,100)
        fr5_A.put([cup[0]+left-right,cup[1]-forward+back], "yn" , "3",obj_height = 90)
        fr5_A.MoveLDelta(0,50,0)       
        fr5_A.MoveLDelta(0,0,200)    
    if sel_num == 4:
        fr5_B.put([cup[0]+left-right,cup[1]-forward+back], "xp" , "3",obj_height = 0)
        fr5_B.MoveLDelta(-50,0,0)       
        fr5_B.MoveLDelta(0,0,200)  
    
def pourwater(object,pour_num):
    '''
        pour_num=1:从左往右倒
        2:倾倒类型为烧杯
        3:倾倒类型为量筒
        4:往漏斗里倒
    '''
    cup = object.copy()
    if pour_num == 1:        #从左往右倒
        fr5_B.pourwater([cup[0]+100,cup[1],100], "yn" , "2" , clockwise=True)
    if pour_num == 2:        #若倾倒类型为烧杯，添加x方向-100的偏移量
        fr5_B.pourwater([cup[0]-100,cup[1],100], "yn" , "2" )
    if pour_num == 3:        #若倾倒类型为量筒，添加x方向-150的偏移量
        fr5_B.pourwater([cup[0]-150,cup[1],100], "yn" , "3" )
    if pour_num == 4:
        fr5_B.pourwater([cup[0]+100,cup[1],280], "yn" , "2",clockwise=True)
    # placeObjectOn([0,-700],left=300,sel_num=2)
    
def stir(object):
    cup = object.copy()
    fr5_B.pick(cup, "yn", "2", False, is_display=False)
    fr5_B.Go_to_start_zone(open=0)
    fr5_B.robot.MoveL([502.577, -423.656, 210.346, 90.0, 0.0, 0]
                    ,0
                    ,0)
    fr5_B.MoveLDelta(0,0,200)
    fr5_B.robot.SetDO(4,1) 
    input("stiring...press enter to continue")
    fr5_B.robot.SetDO(4,0)
    fr5_B.MoveLDelta(0,0,-230)
    fr5_B.MoveLDelta(-100,0,0)
    
def fire():
    logger.info('open lamp head')
    fr5_B.robot.MoveL([466.0,-297.0,238.0,126.0,2.0,90.0],0,0)

    fr5_B.MoveLDelta(z=-40)
    # # close gripper
    fr5_B.MoveGripper(1, 20, 50, 10, 10000, 1)
    fr5_B.MoveLDelta(x=10,z=100)
    fr5_B.MoveLDelta(y=-100)
    fr5_B.MoveLDelta(z=-170)
    # open gripper
    fr5_B.MoveGripper(1, 50, 50, 10, 10000, 1)
    time.sleep(5)
    fr5_B.Go_to_start_zone()
    # closefire()
    
    fr5_A.robot.MoveL([539.439, -355.589, 398.438 , 90.0, 0, 0]
                    ,0
                    ,0)
    time.sleep(1)
    fr5_A.robot.MoveL([542.439, -458.589, 398.438 , 89.0, 0, 0]
                    ,0
                    ,0)
    # 抓取酒精灯
    pick([500,-300],sel_num=2)
    fr5_B.Go_to_start_zone(open=0)
    fr5_B.robot.MoveL([-476.129, -333.229, 350.604, 90,0 ,0 ],
                      0,
                      0)
    fr5_B.MoveLDelta(0,-100,5)
    time.sleep(3)
    # close gripper fire!!!
    fr5_A.MoveGripper(1, 0, 50, 10, 10000, 1)
    time.sleep(3)
    # open gripper
    fr5_A.MoveGripper(1, 100, 50, 10, 10000, 1)
    time.sleep(3)
    fr5_A.robot.MoveL([539.439, -355.589, 398.438 , 90.0, 0, 0]
                    ,0
                    ,0)
    
    fr5_B.MoveLDelta(0,200,0)
    fr5_B.MoveLDelta(0,0,-200)
    fr5_B.robot.MoveL([-257.489,-383.409,176.573,90,0,0]
                      ,0
                      ,0)
    fr5_B.MoveLDelta(0,-40,0)
    time.sleep(5)
    input("heating....press enter to continue")
    fr5_B.MoveLDelta(0,200,0)
    
def closefire():
    fr5_B.MoveGripper(1, 50, 50, 10, 10000, 1)
    fr5_B.robot.MoveL([466.0,-397.0,228.0,126.0,2.0,90.0],0,0)
    fr5_B.MoveLDelta(x=20,z=-120)
    fr5_B.MoveGripper(1, 20, 50, 10, 10000, 1)
    time.sleep(2)
    fr5_B.MoveLDelta(z=200)
    fr5_B.robot.MoveL([466.0,-297.0,238.0,126.0,2.0,90.0],0,0)
    fr5_B.MoveLDelta(z=-40)
    fr5_B.MoveLDelta(z=40)
    fr5_B.MoveLDelta(z=-40)
    fr5_B.MoveGripper(1, 50, 50, 10, 10000, 1)
    fr5_B.MoveLDelta(z=100)
    
    fr5_B.Go_to_start_zone()

if __name__ == '__main__':
    # fr5_B.MoveGripper(1, 100, 50, 10, 10000, 1)
    fr5_A.dou_go_start(fr5_B)
    # fire()
    
    # 以左侧为置物区
    water = [600, -100]
    crude_salt = [600, 0]
    funnel = [-500,-500]
    cup_guolv = [-500,-400]
    operate_area = [200,-700]
    logger.info("------------开始粗盐提纯实验------------")
    
    # # 把水放到中间
    # logger.info("把水放到操作区")
    # pick(water,sel_num=2)
    # fr5_B.Go_to_start_zone(open=0)
    # placeObjectOn(operate_area, sel_num=2)
    # fr5_B.Go_to_start_zone()
    
    
    # # 把粗盐倒到水里
    # # 抓取粗盐
    # logger.info("抓取粗盐")
    # pick(crude_salt,sel_num=2)
    # fr5_B.Go_to_start_zone(open=0)
    
    # # 把粗盐到在水里
    # logger.info("把粗盐到在水里")
    # pourwater(operate_area, pour_num=1)
    # fr5_B.put(crude_salt, "xp" , "3",obj_height = 0)
    # fr5_B.MoveL(-50,0,0)
    # fr5_B.Go_to_start_zone()
    
    
    # # 搅拌粗盐水
    # logger.info("搅拌粗盐水")
    # stir(operate_area)          
    # fr5_B.Go_to_start_zone(open=0)  
    
    # logger.info("把粗盐水放回操作区")
    # placeObjectOn(operate_area,sel_num=2)
    # fr5_B.Go_to_start_zone()
    
    # logger.info("抓取过滤用烧杯")
    # pick(cup_guolv,sel_num=1)
    # fr5_A.Go_to_start_zone(open=0)
    
    # logger.info("把过滤用烧杯放在操作区")
    # placeObjectOn([0,-600],sel_num=1)
    # fr5_A.Go_to_start_zone()
    
    logger.info("左边机械臂拿漏斗")
    pick(funnel,sel_num=5)
    fr5_A.Go_to_start_zone(open=0)
    fr5_A.MoveLDelta(0,-200,0)
    fr5_A.MoveLDelta(0,0,-200)
    
    
    # 把粗盐水溶液倒入漏斗中
    logger.info("把粗盐水溶液倒入漏斗中")
    pick(operate_area,sel_num=3)
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700],pour_num=4)
    
    # 把粗盐水溶液的烧杯放回原位
    logger.info("把粗盐水溶液的烧杯放回原位")
    placeObjectOn(water,sel_num=4)
    fr5_B.Go_to_start_zone()
    
    # --------------过滤-------------、
    logger.info("过滤中....")
    time.sleep(5)
    input("过滤中...")
    
    # # 放回漏斗
    logger.info("放回漏斗")
    placeObjectOn(funnel,sel_num=3)
    fr5_A.Go_to_start_zone()
        
    # # 把过滤后的溶液倒入坩埚中
    logger.info("抓取过滤后的溶液")
    pick([0,-700],sel_num=3)
    fr5_B.Go_to_start_zone(open=0)
    
    logger.info("把过滤后的溶液倒入坩埚中")
    pourwater([-245,-650],pour_num=4)
    placeObjectOn([0,-700],sel_num=2)
    fr5_B.Go_to_start_zone()
    
    logger.info("把杯子放回操作区")
    pick([0,-600],sel_num=1)
    fr5_A.Go_to_start_zone(open=0)
    placeObjectOn(cup_guolv,sel_num=1)
    fr5_A.dou_go_start(fr5_B)
    
    
    # # # fire
    logger.info("点燃酒精灯")
    fire()
    
    
    fr5_B.Go_to_start_zone(open=0)
    placeObjectOn([500,-300],sel_num=4)
    fr5_A.dou_go_start(fr5_B)
    
    closefire()


