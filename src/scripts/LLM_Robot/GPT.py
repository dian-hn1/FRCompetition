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
import openai
global fr5
fr5 = HNchemistry(2)

openai.api_key = ""     # 删除api_key以除去隐私信息
proxy = {
'http': 'http://127.0.0.1:7890',
'https': 'http://127.0.0.1:7890'
}

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

def pick(object):
    cup = object.copy()
    fr5.pick(cup, "xn" , "3" , False, is_display = False)
    
def placeObjectOn(object,left=0,right=0,forward=0,back=0):
    cup = object.copy()
    fr5.put([cup[0]+left-right,cup[1]-forward+back], "yn" , "3",obj_height = 0)
    
def pourwater(object):
    cup = object.copy()
    fr5.pourwater([cup[0],cup[1],100], "yn" , "2" )
    placeObjectOn(cup0, left=0, right=0, forward=0, back=0)

if __name__ == '__main__':
    cup0 = [0,-700]
    cup1 = [150,-700]
    # fr5 = fr5robot(2)
    fr5.Go_to_start_zone()
    pick(cup0)
    pourwater(cup1)
    placeObjectOn(cup0)
    
    fr5.Go_to_start_zone()
    
    
    # 使用GPT驱动机械臂
    openai.proxy = proxy
    file = open('/home/wangzy/FR5_exp/fr5_gpt/src/scripts/promp.txt', 'r')
    promp = file.read()
    conversation = [{"role": "user", "content": promp}]
    
    while True:
        user_input = input("You:")
        if user_input == "exit":
            break
        conversation.append({"role": "user", "content": user_input})
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo", 
            messages=conversation
        )
        
        reply = completion.choices[0]['message']['content']
        print("Robot:",reply)
        conversation.append({"role": "assistant", "content": reply})

        exe = input("Execute the code? (y/n)")
        if exe == "y":
            if re.search(r"```python\n(.*?)\n```", reply, re.DOTALL):
                code = re.search(r"```python\n(.*?)\n```", reply, re.DOTALL).group(1)
            else:
                code = reply
            execute_code(code)
            fr5.Go_to_start_zone()
    