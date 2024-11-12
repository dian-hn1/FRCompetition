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
import base64
import requests
global fr5
# fr5 = HNchemistry(2)

openai.api_key = ""   # 删除api_key以除去隐私信息
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

def encode_image(image_path):
    # Function to encode the image
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

if __name__ == '__main__':
    # 使用GPT驱动机械臂
    openai.proxy = proxy
    file = open('/home/wangzy/FR5_exp/fr5_gpt/src/scripts/promp.txt', 'r')
    promp = file.read()
    water_figure = encode_image("/home/wangzy/FR5_exp/fr5_gpt/src/scripts/LLM_Robot/figure/water.jpg")
    nowater_figure = encode_image("/home/wangzy/FR5_exp/fr5_gpt/src/scripts/LLM_Robot/figure/nowater.jpg")
    testwater_figure = encode_image("/home/wangzy/FR5_exp/fr5_gpt/src/scripts/LLM_Robot/figure/testwater.jpg")
    conversation = [{"role": "user", "content": [
        {
          "type": "text",
          "text": "There is liquid in the funnel in this figure."
        },
        {
          "type": "image_url",
          "image_url": {
            "url": f"data:image/jpeg;base64,{water_figure}"
          }
        },

        {
          "type": "text",
          "text": "There is no liquid in the funnel in the second figure.The white part is the sand core for filtration."
        },
        {
          "type": "image_url",
          "image_url": {
            "url": f"data:image/jpeg;base64,{nowater_figure}"
          }
        },

        {
          "type": "text",
          "text": "So is there any liquid in the funnel in next figure? You can only answer yes or no without any explaination."
        },
        {
          "type": "image_url",
          "image_url": {
            "url": f"data:image/jpeg;base64,{testwater_figure}"
          }
        }
      ]
      }]
    
    
    while True:
        user_input = input("You:")
        if user_input == "exit":
            break
        conversation.append({"role": "user", "content": user_input})
        completion = openai.ChatCompletion.create(
            model="gpt-4o", 
            messages=conversation
        )
        
        reply = completion.choices[0]['message']['content']
        print("Robot:",reply)
        conversation.append({"role": "assistant", "content": reply})

        # exe = input("Execute the code? (y/n)")
        # if exe == "y":
        #     if re.search(r"```python\n(.*?)\n```", reply, re.DOTALL):
        #         code = re.search(r"```python\n(.*?)\n```", reply, re.DOTALL).group(1)
        #     else:
        #         code = reply
        #     execute_code(code)
        #     fr5.Go_to_start_zone()
    