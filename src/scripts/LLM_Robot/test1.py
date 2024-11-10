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
import threading
# import openai
# global fr5
global fr5_A
global fr5_B
fr5_A = HNchemistry(1)      #192.168.59.6（左侧机械臂）
fr5_B = HNchemistry(2)      #192.168.58.6(右侧机械臂)

cup_dianfen = [-500,0]
cup_reshui = [-400,0]
cup_bingersuan = [-400,-100]
cup_liusuanmeng = [-500,-100]
cup_shui50g = [-400,-200]
cup_diansuanjia = [-500,-200]
cup_h2o2 = [-400,-300]

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
        fr5_B.MoveLDelta(0,0,200)
        
    
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
    # fr5_B.robot.MoveL([0.6638992428779602, -250.441207885742, 400.4645690917968, 90.00076293945312, 0.001102424575947225, -0.03089224360883236]
    #                 ,0,0)
    fr5_B.robot.MoveL([500, -450.7618408203125, 199.9986572265625, 90.11894226074217, 0.10213880985975267, -0.05570357292890549]
                    ,0
                    ,0)
    fr5_B.MoveLDelta(0,0,230)
    fr5_B.robot.SetDO(0,1) 
    time.sleep(5)
    fr5_B.robot.SetDO(0,0)
    fr5_B.MoveLDelta(0,0,-230)

def threading_1(x,y):
    pick([200,-600],sel_num=1)         #左爪抓取淀粉空杯，并放置于（x，y）处
    fr5_A.Go_to_start_zone(open=0)
    placeObjectOn([x,y],sel_num=1)
    fr5_A.Go_to_start_zone()

def threading_2_1():
    stir([0,-700])          #右爪将加入淀粉后的热水搅拌
    fr5_B.Go_to_start_zone(open=0)    
    placeObjectOn(cup_reshui,left=400,forward=700,sel_num=2)
    fr5_B.Go_to_start_zone()
    
def threading_2_2():
    pick(cup_liusuanmeng,2)   #抓取硫酸锰
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #将硫酸锰加入淀粉溶液中,并放置空杯
    fr5_B.Go_to_start_zone()
    
def threading_2_3():
    time.sleep(13)
    stir([-100,-700])          #搅拌碘酸钾溶液
    fr5_B.Go_to_start_zone(open=0)
    placeObjectOn(cup_shui50g,left=300,forward=500,sel_num=2)
    fr5_B.Go_to_start_zone()
    
def threading_2_4():
    pick(cup_h2o2,sel_num=2)      #抓取硫酸酸化的过氧化氢溶液
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #倒入混合溶液中
    fr5_B.Go_to_start_zone()
    
def run_threads(thread1_func,thread2_func,thread1_args=(),thread2_args=()):
    thread1 = threading.Thread(target=thread1_func,args=thread1_args)
    thread2 = threading.Thread(target=thread2_func,args=thread2_args)
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()

def main():
    fr5_A.dou_go_start(fr5_B)
    
    # cup_dianfen = [-500,0]
    # cup_reshui = [-400,0]
    # cup_bingersuan = [-400,-100]
    # cup_liusuanmeng = [-500,-100]
    # cup_shui50g = [-400,-200]
    # cup_diansuanjia = [-500,-200]
    # cup_h2o2 = [-400,-300]
    # cup_shui30g = [-500,-300]
    # bigcup = [-400,-400]
    
    pick(cup_reshui,sel_num=2)      #抓取热水，放置于（0，-700）后归位
    fr5_B.Go_to_start_zone(open=0)
    placeObjectOn(cup_reshui,left=400,forward=700,sel_num=2)
    fr5_B.Go_to_start_zone()
    
    pick(cup_dianfen,sel_num=2)       #抓取淀粉
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #将淀粉加入热水，并放置空杯
    fr5_B.Go_to_start_zone()

    run_threads(threading_1,threading_2_1,(-500,-500),())
    
    pick(cup_bingersuan,sel_num=2)    #抓取丙二酸
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #将丙二酸加入淀粉溶液中,并放置空杯
    fr5_B.Go_to_start_zone()

    run_threads(threading_1,threading_2_2,(-500,-400),())
    
    run_threads(threading_1,threading_2_1,(-500,-300),()) 
    
    pick(cup_shui50g,sel_num=2)       #抓取50ml水
    fr5_B.Go_to_start_zone(open=0)
    placeObjectOn(cup_shui50g,left=300,forward=500,sel_num=2)     #放置于（-100，-700）
    fr5_B.Go_to_start_zone()
    
    pick(cup_diansuanjia,sel_num=2)   #抓取碘酸钾
    fr5_B.Go_to_start_zone(open=0)
    pourwater([-100,-700], pour_num = 2)    #将碘酸钾加入50ml水中
    fr5_B.Go_to_start_zone()
    
    run_threads(threading_1,threading_2_3,(-400,-500),())
    
    pick([-100,-700],sel_num=2)
    fr5_B.Go_to_start_zone(open=0)
    pourwater([0,-700], pour_num = 2)    #将碘酸钾溶液加入到第一杯（淀粉、丙二酸、硫酸锰）溶液中,并放置空杯
    fr5_B.Go_to_start_zone()

    run_threads(threading_1,threading_2_4,(-400,-400),())

if __name__ == '__main__':
    main()
    