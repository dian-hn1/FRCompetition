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
    cup = object.copy()
    #sel_num=1时操作左侧机械臂；为2时操作右侧机械臂
    if sel_num == 1:
        fr5_A.pick(cup, "yn" , "3" , False, is_display = False)
    if sel_num == 2:
        fr5_B.pick(cup, "yn" , "3" , False, is_display = False)
    if sel_num == 3:
        fr5_B.pick(cup, "xn" , "3" , False, is_display = False)
    
def placeObjectOn(object,left=0,right=0,forward=0,back=0,sel_num=1):
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
        fr5_B.MoveLDelta(0,0,20      0)
        
    
def pourwater(object,pour_num):
    cup = object.copy()
    if pour_num == 2:        #若倾倒类型为烧杯，添加x方向-100的偏移量
        fr5_B.pourwater([cup[0]-100,cup[1],100], "yn" , "2" )
    if pour_num == 3:        #若倾倒类型为量筒，添加x方向-150的偏移量
        fr5_B.pourwater([cup[0]-150,cup[1],100], "yn" , "3" )
    placeObjectOn([0,-700],right=200,sel_num=2)
    
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
    

if __name__ == '__main__':
    # cup0 = [0,-700]
    # cup1 = [150,-700]
    # fr5 = fr5robot(2)
    fr5_A.dou_go_start(fr5_B)
    
    # pick(cup0)
    # stir(cup0)
    # pourwater(cup1)
    # placeObjectOn(cup0)
    
    # for debug
    # 以右侧为置物台
    cup_dianfen = [-500,0]
    cup_reshui = [-400,0]
    cup_bingersuan = [-400,-100]
    cup_liusuanmeng = [-500,-100]
    cup_shui50g = [-400,-200]
    cup_diansuanjia = [-500,-200]
    cup_h2o2 = [-400,-300]
    cup_shui30g = [-500,-300]
    bigcup = [-400,-400]
    
    # 测试左爪
    # pick([200,-600],1)
    # fr5_A.Go_to_start_zone(open=0)
    # placeObjectOn([-500,-300],sel_num=1)
    # fr5_A.Go_to_start_zone()
    # 测试右爪
    # pick([200,-700],2)
    # fr5_B.Go_to_start_zone(open=0)
    # placeObjectOn([-200,-600],sel_num=2)
    # fr5_B.Go_to_start_zone()

    
    # pick(cup_reshui)        #抓取热水，放置于（0，-800）后归位
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn(cup_reshui,left=400,forward=800)
    # fr5.Go_to_start_zone()
    
    # stir([0,-700])          #将加入淀粉后的热水搅拌
    # fr5_B.Go_to_start_zone(open=0)
    
    logger.info("抓取热水")
    pick(cup_reshui,sel_num=2)      #抓取热水，放置于（0，-700）后归位
    fr5_B.Go_to_start_zone(open=0)
    logger.info("放置热水")
    placeObjectOn(cup_reshui,left=400,forward=700,sel_num=2)
    fr5_B.Go_to_start_zone()
    
    
    # pick(cup_dianfen)       #抓取淀粉
    # fr5.Go_to_start_zone(open=0)
    # pourwater([0,-800], sel_num = 2)    #将淀粉加入热水
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn([0,-800],right=300)   #将空淀粉烧杯放置于（-300，-800）
    # fr5.Go_to_start_zone()
    
    logger.info("抓取淀粉")
    pick(cup_dianfen,sel_num=2)       #抓取淀粉
    fr5_B.Go_to_start_zone(open=0)
    logger.info("将淀粉加入热水中")
    pourwater([0,-700], pour_num = 2)    #将淀粉加入热水
    fr5_B.Go_to_start_zone()
    # fr5_B.Go_to_start_zone(open=0)
    # placeObjectOn([0,-700],right=200,sel_num=2)   #将空淀粉烧杯放置于（-200，-700）
    # fr5_B.Go_to_start_zone()
    
    logger.info("抓取空杯")
    pick([200,-600],sel_num=1)         #左爪抓取淀粉空杯
    fr5_A.Go_to_start_zone(open=0)
    logger.info("放置空杯")
    placeObjectOn([-500,-500],sel_num=1)
    fr5_A.Go_to_start_zone()
    
    
    # stir([0,-800])          #将加入淀粉后的热水搅拌
    # fr5.Go_to_start_zone(open=0)    
    # placeObjectOn(cup_reshui,left=400,forward=800)
    # fr5.Go_to_start_zone()
    
    logger.info("搅拌淀粉溶液")
    stir([0,-700])          #将加入淀粉后的热水搅拌
    fr5_B.Go_to_start_zone(open=0)    
    placeObjectOn(cup_reshui,left=400,forward=700,sel_num=2)
    fr5_B.Go_to_start_zone()
    
    
    # pick(cup_bingersuan)    #抓取丙二酸
    # fr5.Go_to_start_zone(open=0)
    # pourwater([0,-800], sel_num = 2)    #将丙二酸加入淀粉溶液中
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn([0,-800],right=200)   #将空的丙二酸烧杯放置于（-200，-800）
    # fr5.Go_to_start_zone()
    
    logger.info("抓取丙二酸")
    pick(cup_bingersuan,sel_num=2)    #抓取丙二酸
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #将丙二酸加入淀粉溶液中
    fr5_B.Go_to_start_zone()
    # placeObjectOn([0,-700],right=200,sel_num=2)   #将空的丙二酸烧杯放置于（-200，-700）
    # fr5_B.Go_to_start_zone()
    
    logger.info("抓取丙二酸空杯")
    pick([200,-600],1)         #左爪抓取丙二酸空杯
    fr5_A.Go_to_start_zone(open=0)
    placeObjectOn([-500,-400],sel_num=1)
    fr5_A.Go_to_start_zone()
    
    
    # pick(cup_liusuanmeng)   #抓取硫酸锰
    # fr5.Go_to_start_zone(open=0)
    # pourwater([0,-800], sel_num = 2)    #将硫酸锰加入淀粉溶液中
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn([0,-800],right=300,back=100)  #将空的硫酸锰烧杯放置于（-200，-700）
    # fr5.Go_to_start_zone()
    
    logger.info("抓取硫酸锰")
    pick(cup_liusuanmeng,sel_num=2)   #抓取硫酸锰
    fr5_B.Go_to_start_zone(open=0)
    +, pour_num = 2)    #将硫酸锰加入淀粉溶液中
    fr5_B.Go_to_start_zone()
    # placeObjectOn([0,-700],right=200,sel_num=2)  #将空的硫酸锰烧杯放置于（-200，-700）
    # fr5_B.Go_to_start_zone()
    
    logger.info("抓取硫酸锰空杯")
    pick([200,-600],1)         #左爪抓取硫酸锰空杯
    fr5_A.Go_to_start_zone(open=0)
    placeObjectOn([-500,-300],sel_num=1)
    fr5_A.Go_to_start_zone()
    
    
    # stir([0,-800])          #搅拌加入了丙二酸和硫酸锰的淀粉溶液
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn(cup_reshui,left=400,forward=800)
    # fr5.Go_to_start_zone()
    
    logger.info("搅拌溶液")
    stir([0,-700])          #搅拌加入了丙二酸和硫酸锰的淀粉溶液
    fr5_B.Go_to_start_zone(open=0)
    placeObjectOn(cup_reshui,left=400,forward=700,sel_num=2)
    fr5_B.Go_to_start_zone()
    
    
    # pick(cup_shui50g)       #抓取50ml水
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn(cup_shui50g,left=400,forward=500)     #放置于（0，-700）
    # fr5.Go_to_start_zone()
    
    logger.info("抓取水")
    pick(cup_shui50g,sel_num=2)       #抓取50ml水
    fr5_B.Go_to_start_zone(open=0)
    placeObjectOn(cup_shui50g,left=300,forward=500,sel_num=2)     #放置于（-100，-700）
    fr5_B.Go_to_start_zone()
    
    
    # pick(cup_diansuanjia)   #抓取碘酸钾
    # fr5.Go_to_start_zone(open=0)
    # pourwater([0,-700], sel_num = 2)    #将碘酸钾加入50ml水中
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn([0,-700],right=200)   #将空的碘酸钾烧杯放置于（-200，-700）
    # fr5.Go_to_start_zone()
    
    logger.info("抓取碘酸钾")
    pick(cup_diansuanjia,sel_num=2)   #抓取碘酸钾
    fr5_B.Go_to_start_zone(open=0)
    pourwater([-100,-700], pour_num = 2)    #将碘酸钾加入50ml水中
    fr5_B.Go_to_start_zone()
    # placeObjectOn([0,-700],right=200,sel_num=2)   #将空的碘酸钾烧杯放置于（-200，-700）
    # fr5_B.Go_to_start_zone()
    
    logger.info("抓取碘酸钾空杯")
    pick([200,-600],1)         #左爪抓取碘酸钾空杯
    fr5_A.Go_to_start_zone(open=0)
    placeObjectOn([-400,-500],sel_num=1)
    fr5_A.Go_to_start_zone()
    
    # stir([0,-700])          #搅拌碘酸钾溶液
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn(cup_shui50g,left=400,forward=500)
    # fr5.Go_to_start_zone()
    
    logger.info("搅拌碘酸钾溶液")
    stir([-100,-700])          #搅拌碘酸钾溶液
    fr5_B.Go_to_start_zone(open=0)
    placeObjectOn(cup_shui50g,left=300,forward=500,sel_num=2)
    fr5_B.Go_to_start_zone()
    
    
    # pick([0,-700])
    # fr5.Go_to_start_zone(open=0)
    # pourwater([0,-800], sel_num = 2)    #将碘酸钾溶液加入到第一杯（淀粉、丙二酸、硫酸锰）溶液中
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn([0,-700],right=300,back=100)      #将空的碘酸钾溶液烧杯放置在（-300，-600）
    # fr5.Go_to_start_zone()
    
    logger.info("混合液体")
    pick([-100,-700],sel_num=2)
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #将碘酸钾溶液加入到第一杯（淀粉、丙二酸、硫酸锰）溶液中
    fr5_B.Go_to_start_zone()
    # placeObjectOn([0,-700],right=200,sel_num=2)      #将空的碘酸钾溶液烧杯放置在（-200，-700）
    # fr5_B.Go_to_start_zone()
    
    logger.info("放置空杯")
    pick([200,-600],1)         #左爪抓取碘酸钾溶液空杯
    fr5_A.Go_to_start_zone(open=0)
    placeObjectOn([-400,-400],sel_num=1)
    fr5_A.Go_to_start_zone()
    
    # pick(cup_h2o2)      #抓取硫酸酸化的过氧化氢溶液
    # fr5.Go_to_start_zone(open=0)
    # pourwater([0,-800], sel_num = 2)    #倒入混合溶液中
    # fr5.Go_to_start_zone(open=0)
    # placeObjectOn([0,-700],right=200,back=100)      #将空的过氧化氢溶液烧杯放置于（-200，-600） 
    # fr5.Go_to_start_zone()
    
    logger.info("抓取过氧化氢溶液")
    pick(cup_h2o2,sel_num=2)      #抓取硫酸酸化的过氧化氢溶液
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #倒入混合溶液中
    fr5_B.Go_to_start_zone()
    # placeObjectOn([0,-700],right=200,sel_num=2)      #将空的过氧化氢溶液烧杯放置于（-200，-700） 
    # fr5_B.Go_to_start_zone()
    
    # pick([200,-600],1)         #左爪抓取碘酸钾溶液空杯
    # fr5_A.Go_to_start_zone(open=0)
    # placeObjectOn([-400,-300],sel_num=1)
    # fr5_A.Go_to_start_zone()
    
    
    
    
    
    # fr5.MoveL(500,-250,200)
    # fr5.robot.SetDO(0,1)
    # time.sleep(2)
    # fr5.robot.SetDO(0,0)
    
    # fr5.Go_to_start_zone()

    
    
    