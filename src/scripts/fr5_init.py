'''
法奥新版SDK，适配fr5机械臂，python=3.10
update date 2024-04-11
'''

import sys

# sys.path.append("/home/zjh/FR5_exp/script")
import copy
import time
import Robot
import numpy as np
import rospy
from std_msgs.msg import Int32
import threading
import struct
from serial.tools import list_ports
import serial
import time

j3_auto_weight = [-49.458, -55.125, -124.682, -180.193, -49.458, 0.000]# 这个是机械臂复位的J
eP1=[0.000,0.000,0.000,0.000]
dP1=[1.000,1.000,1.000,1.000,1.000,1.000]
oP1=[0.000,0.000,0.000,0.000,0.000,0.000]
class fr5robot:
    def __init__(self, index=1):
        # Initialize the ROS node

        # 初始化变量
        if index == 1:
            self.robot = Robot.RPC('192.168.59.2')
        if index == 2:
            self.robot = Robot.RPC('192.168.58.2')
            
        self.index = index
        self.robot.SetAO(0, 0.0, 0)
        # 夹爪初始化 大寰
        self.robot.SetGripperConfig(4, 0, 0, 1)
        # self.robot.ActGripper(1, 0)
        # time.sleep(0.5)
        self.robot.ActGripper(1, 1)
        

        # time.sleep(1)
        # self.robot.MoveGripper(1, 100, 50, 10, 10000, 1)
        # time.sleep(1)

        print("夹爪初始化完成")
        rospy.init_node('fr5_main', anonymous=True)
        self.pub_gripper1 = rospy.Publisher('pub_gripper1', Int32, queue_size=0)
        self.pub_gripper2 = rospy.Publisher('pub_gripper2', Int32, queue_size=0)

    def MoveGripper(self,index,pos,speed,force,maxtime,block):
        '''
        index:夹爪编号；
        pos:位置百分比，范围[0~100]；
        speed:速度百分比，范围[0~100];
        force:力矩百分比，范围[0~100]；
        maxtime:最大等待时间，范围[0~30000]，单位[ms]；
        block:0-阻塞，1-非阻塞。
        '''
        if self.index == 1:
            self.robot.MoveGripper(index,pos,speed,force,maxtime,block)
            gripper_pos = Int32()
            gripper_pos.data = pos
            self.pub_gripper2.publish(gripper_pos)
        elif self.index == 2:
            self.robot.MoveGripper(index,pos,speed,force,maxtime,block)
            gripper_pos = Int32()
            gripper_pos.data = pos
            self.pub_gripper1.publish(gripper_pos)
        else :
            exit()

    def Go_to_start_zone(self,v = 30.0, open = 1):
        '''
            机械臂复位
        '''
        self.point_safe_move([0.0, -250.0, 400.0, 90.0, 0.0, 0.0], v, 200.0)
        if open:
            self.MoveGripper(1, 100, 50, 10, 10000, 1)

    def dou_go_start(self, fr5_B, v=50.0):
        '''
            两个机械臂同时复位
        '''
        self.Go_to_start_zone(v)
        time.sleep(1)
        fr5_B.Go_to_start_zone(v)

    def MoveL(self, x=0.000, y=0.000, z=0.000, movespeed=100.0):
        '''
            机械臂直线运动
        '''
        eP1 = [0.000, 0.000, 0.000, 0.000]
        dP1 = [x, y, z, 0.000, 0.000, 0.000]
        pos_now = self.robot.GetActualToolFlangePose(0)
        # print('pos_now:',pos_now)

        while( type(pos_now) != tuple):
            pos_now = self.robot.GetActualToolFlangePose(0)
            time.sleep(0.5)
        pos_now = pos_now[1]

        pos_now[0] = pos_now[0] + x
        pos_now[1] = pos_now[1] + y
        pos_now[2] = pos_now[2] + z
        ret = self.robot.MoveL(
            pos_now,
            0,
            0,
            blendR = -1.0,
        )
        # print('cuowuma',ret)

    def point_safe_move(self, start_catch_position, v=60.0, height=250.0, last_v = 0):  
        '''
            机械臂安全运动到指定位置
        '''

        end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
        # print(type(end_height_from_sdk) )
        # 如果查询失败的话，重新查询
        while( type(end_height_from_sdk) != tuple):
            end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
            print('when executing point_safe_move,failed to get end_height_from_sdk1')
            time.sleep(0.5)
        # print('end_height_from_sdk:',end_height_from_sdk)
        if len(end_height_from_sdk) == 2:
            end_height = end_height_from_sdk[1]
        if end_height[2] < height:
            # pass
            self.MoveL(0.0, 0.0, (max(height, start_catch_position[2]) - end_height[2] ), v)
        time.sleep(1)
        middle_pos = copy.deepcopy(start_catch_position)

        end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
        # 如果查询失败的话，重新查询
        while( type(end_height_from_sdk) != tuple):
            end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
            print('when executing point_safe_move,failed to get end_height_from_sdk2')
            time.sleep(0.5)
        # print('end_height_from_sdk:',end_height_from_sdk)
        if len(end_height_from_sdk) == 2:
            end_height = end_height_from_sdk[1]
        if (
            end_height[2]  < height
            and start_catch_position[2] < height
        ):
            middle_pos[2] = height
        else:
            middle_pos[2] = max(end_height[2] , start_catch_position[2])
        p3_auto_weight = self.robot.GetForwardKin(j3_auto_weight)
        while( type(p3_auto_weight) != tuple):
                p3_auto_weight = self.robot.GetForwardKin(j3_auto_weight)
                time.sleep(0.1)
        p3_auto_weight = p3_auto_weight[1]
        self.robot.MoveJ(j3_auto_weight,0,0,p3_auto_weight,20.0,0,100.0,eP1,-1.0,0,oP1)
        # self.robot.MoveCart(middle_pos, 0, 0)

        time.sleep(1)
        end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
        # 如果查询失败的话，重新查询
        while( type(end_height_from_sdk) != tuple):
            end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
            print('when executing point_safe_move,failed to get end_height_from_sdk3')
            time.sleep(0.5)
        # print('end_height_from_sdk:',end_height_from_sdk)
        if len(end_height_from_sdk) == 2:
            end_height = end_height_from_sdk[1]
        if last_v == 0:
            self.MoveL(
                0.0, 0.0, (start_catch_position[2] - end_height[2] ), v
        )
        else:
            self.MoveL(
                0.0, 0.0, (start_catch_position[2] - end_height[2] ), last_v
        )

    def Safe_move(self, start_catch_position, dir, v=30.0):
        temp_path = []
        start_interpolation_path = []
        # get end pos
        start_position = self.robot.GetActualToolFlangePose(0)
        # print(start_position)
        # 进行处理 把首位0删除
        start_position = start_position[-6:]
        # save initial path point
        start_interpolation_path += [
            start_position,
        ]
        # add one path ---point z upto 250.0 rxryrz unfollowed
        temp_path = Add_path(
            start_position,
            start_position[0],
            start_position[1],
            250.0,
            start_position[3],
            start_position[4],
            start_position[5],
        )
        start_interpolation_path += [temp_path]
        if dir == "yn":
            print()
            # add one path ---point xy move to object rxryrz followed(y bias = 100.0)
            temp_path = Add_path(
                temp_path,
                start_catch_position[0],
                start_catch_position[1] + 100.0,
                250.0,
                start_catch_position[3],
                start_catch_position[4],
                start_catch_position[5],
            )
            start_interpolation_path += [temp_path]
            # add one path ---point z down to object
            temp_path = Add_path(
                temp_path,
                temp_path[0],
                temp_path[1],
                start_catch_position[2],
                temp_path[3],
                temp_path[4],
                temp_path[5],
            )
            start_interpolation_path += [temp_path]
            # add one path ---final position
            temp_path = Add_path(
                temp_path,
                temp_path[0],
                temp_path[1] - 100.0,
                start_catch_position[2],
                temp_path[3],
                temp_path[4],
                temp_path[5],
            )
            start_interpolation_path += [temp_path]
        elif dir == "xn":
            print()
            # add one path ---point xy move to object rxryrz followed(y bias = 100.0)
            temp_path = Add_path(
                temp_path,
                start_catch_position[0] + 100.0,
                start_catch_position[1],
                250.0,
                start_catch_position[3],
                start_catch_position[4],
                start_catch_position[5],
            )
            start_interpolation_path += [temp_path]
            # add one path ---point z down to object
            temp_path = Add_path(
                temp_path,
                temp_path[0],
                temp_path[1],
                start_catch_position[2],
                temp_path[3],
                temp_path[4],
                temp_path[5],
            )
            start_interpolation_path += [temp_path]
            # add one path ---final position
            temp_path = Add_path(
                temp_path,
                temp_path[0] - 100.0,
                temp_path[1],
                start_catch_position[2],
                temp_path[3],
                temp_path[4],
                temp_path[5],
            )
            start_interpolation_path += [temp_path]
        else:
            exit()
        print(start_interpolation_path)
        # 执行移动
        for i in range(len(start_interpolation_path)):
            self.robot.MoveCart(
                start_interpolation_path[i], 0, 0, 0.0, 0.0, v, -1.0, -1
            )

    # 参数为容器半径，容器上平面离夹爪中心高度，角度增量及方向，指令周期，最大旋转角度
    def pour(self, r, h, i=-2, max_angel=90, rate = 100.0, v = 70.0, upright = 1, shake = 1):
        rate /= 100
        types = {
            "100ml": {"diameter": 10, "height": 20},
            "200ml": {"diameter": 15, "height": 25},
            "250ml": {"diameter": 12, "height": 22},
        }
        # 伺服运动参数预置
        t=0.002
        eP0 = [0.000, 0.000, 0.000, 0.000]
        dP0 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
        gain = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # 位姿增量比例系数，仅在增量运动下生效，范围[0~1]
        
        P1 = self.robot.GetActualTCPPose(0)
        J1 = self.robot.GetActualJointPosDegree(0)
        while( type(P1) != tuple):
            P1 = self.robot.GetActualToolFlangePose(0)
            print('when executing pouring,failed to get P1 from sdk')
            time.sleep(0.5)
        P1 = P1[1]
        while( type(J1) != tuple):
            J1 = self.robot.GetActualJointPosDegree(0)
            print('when executing pouring,failed to get J1 from sdk')
            time.sleep(0.5)
        J1 = J1[1]

        # 计算旋转参数
        R = np.sqrt(r**2 + h**2)
        phi = np.arctan(h / r)
        l = np.pi * R / 180  # 弧长（x理论增量）
        # 工具坐标笛卡尔增量
        n_pos = [
            float(2.2 * l * np.sin(phi) * np.sign(i)) * rate,
            float(2.2 * l * np.cos(phi)) * rate,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        joint_pos_difference = 0  # 确保进入倾倒循环
        
        # 在末端关节伺服旋转时，执行空间伺服运动以确保仪器出料口位置稳定
        # 可通过修改单步时延t的大小来调整旋转速度
        while np.abs(joint_pos_difference) < max_angel:
            self.robot.ServoCart(2, n_pos, gain, 0.0, 0.0, t, 0.0, 0.0)  # 工具笛卡尔坐标增量移动

            joint_pos = self.robot.GetActualJointPosDegree(0)
            while( type(joint_pos) != tuple):
                joint_pos = self.robot.GetActualJointPosDegree(0)
                print('when executing pouring,failed to get end_height_from_sdk2')
                time.sleep(0.5)
            joint_pos = joint_pos[1]
            joint_pos[5] = joint_pos[5] + i * rate

            self.robot.ServoJ(joint_pos, 0.0, 0.0, t, 0.0, 0.0)  # 关节角增量移动

            time.sleep(t)

            # 更新差值
            joint_pos_difference = joint_pos[5] - J1[5]

        joint_pos = self.robot.GetActualJointPosDegree(0)
        while( type(joint_pos) != tuple):
            joint_pos = self.robot.GetActualJointPosDegree(0)
            print('when executing pouring,failed to get end_height_from_sdk2')
            time.sleep(0.5)
        joint_pos = joint_pos[1]
        pos_record = self.robot.GetActualTCPPose(0)
        while( type(pos_record) != tuple):
            pos_record = self.robot.GetActualTCPPose(0)
            print('when executing pouring,failed to get pos record')
            time.sleep(0.5)
        pos_record = pos_record[1]
        max_angel = joint_pos[5] + 6.0
        min_angel = joint_pos[5] - 6.0
        shakes = 0
        if shake == 1:
            while shakes < 200:
                self.robot.ServoJ(joint_pos, 0.0, 0.0, t, 0.0, 0.0)
                if joint_pos[5] > max_angel:
                    i = -1
                if joint_pos[5] < min_angel:
                    i = 1
                joint_pos[5] += i

                time.sleep(0.002)
                shakes += 1
            self.robot.MoveCart(pos_record, 0, 0, 0.0, 0.0, v, -1.0, -1)
            
        # 回到倾倒起点，有些场景下可能需要，暂时保留
        # self.MoveL(0.0, 0.0, (300 - self.robot.GetActualTCPPose(0)[3]), 50.0)
        # time.sleep(0.5)
        
        # 是否需要上抬以规避倾倒仪器回归水平位时的碰撞
        if upright == 0:
            return
        else:
            P1 = self.robot.GetActualTCPPose(0)[1]
            P1[2] += 60.0
            if np.abs(P1[5]) > 80.0 and np.abs(P1[5]) < 170.0:
                P1 = P1[0:3]
                P1 += [90.0, 0.0, -90.0]
            else:
                P1 = P1[0:3]
                P1 += [90.0, 0.0, 0.0]
            self.robot.MoveCart(P1, 0, 0, 0.0, 0.0, v, -1.0, -1)
    
    #  蠕动泵控制 on = 0 关闭蠕动泵 on = 1 开启蠕动泵; block = 0 非阻塞 block = 1 阻塞
    #  index = 0 对应蠕动泵A，index = 1对应蠕动泵B
    #  速度采取百分制，范围[0~100]，100对应控制箱模拟量输出10V
    #  注，当前仅机械臂B控制箱有正常模拟量输出，使用时应配合机械臂A
    def pump(self, index, on, speed = 100, block = 1):
        speed = float(speed)
        if on == 0:
            self.robot.SetAO(index,block)    # 关闭蠕动泵
        if on == 1:
            self.robot.SetAO(index,speed,block)    # 开启蠕动泵

    class SimpleController_liquid:
        def __init__(self, robot, set_point, k=95.0):
            self.k = k
            self.set_point = set_point
            self.change_point = set_point/10.0
            self.robot = robot
            self.control_action = 100.0
        
        def rd_ctr(self, on, measured_value=0.0):
            if on == 0:
                self.robot.SetAO(0, 0.0, 0)
            if on == 1:
                error = self.set_point - measured_value
                if error >= self.change_point :
                    self.robot.SetAO(0, 100.0, 0)
                else:
                    self.control_action = (self.k * error) / self.change_point  + 5.0
                    self.robot.SetAO(0, self.control_action, 0)

    def extract_and_convert(hex_data):
        # 确保传入的数据是bytes类型
        if not isinstance(hex_data, bytes) or len(hex_data) != 9:
            raise ValueError("Invalid bytes data provided")

        bytes_data = hex_data[5:7]
        bytes_data += hex_data[3:5]
        
        # 使用struct解包为有符号整数
        result = struct.unpack('>i', bytes_data)[0]

        return result

    def add_liquid(self, set_point):
        print("准备开始正向加料")
        robot = self.robot
        OK=0
        ERROR=-1
        OVERRANGE=-2
        user_com = '/dev/ttyUSB0'
        pd=False
        plist = list(list_ports.comports())

        for port in plist:
            if '/dev/ttyUSB0' in port.device:
                pd=True
                break
        if pd==False:
            print("加液失败：未找到USB串口")
            return ERROR
        SPC=fr5robot.SimpleController_liquid(robot=robot, set_point=set_point)
        command = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B])
        close_protect = bytearray([0x01, 0x06, 0x00, 0x17, 0x00, 0x01, 0xF8, 0x0E])
        zero_set = bytearray([0x01, 0x06, 0x00, 0x16, 0x00, 0x01, 0xA9, 0xCE])
        open_protect = bytearray([0x01, 0x06, 0x00, 0x17, 0x00, 0x00, 0x39, 0xCE])
        try:
            with serial.Serial(user_com, 9600, timeout=1) as ser:
                print("成功连接")
                ser.write(close_protect)
                time.sleep(1)
                response = ser.read(ser.in_waiting)
                if response==close_protect:
                    print("close写保护成功")

                    ser.write(zero_set)
                    time.sleep(1)
                    response = ser.read(ser.in_waiting)
                    if response==zero_set:
                        print("零点调零成功")

                        ser.write(open_protect)
                        time.sleep(1)
                        response = ser.read(ser.in_waiting)
                        if response==open_protect:
                            print("打开写保护成功")

                            while(1):
                                ser.write(command)
                                time.sleep(0.1)
                                response = ser.read(ser.in_waiting)
                                result = fr5robot.extract_and_convert(response)
                                if result>=SPC.set_point:
                                    print("Arrived")
                                    break
                                SPC.rd_ctr(1, result)
                                if result < 0:
                                    res = abs(result)
                                    a = int(res // 100)
                                    b = res - (a * 100)
                                    print(f"Received response: -{a} . {b:02} g")
                                else:
                                    a = int(result // 100)
                                    b = result - (a * 100)
                                    print(f"Received response: {a} . {b:02} g")
                                print("Running speed:", SPC.control_action)
                            else:
                                print("打开写保护失败")
                        else:
                            print("打开写保护失败")
                    else:
                        print("零点调零失败")
        except Exception as e:
            print("Error:", str(e))

        SPC.rd_ctr(0)
        tot=0
        for i in range(5):
            try:
                with serial.Serial(user_com, 9600, timeout=1) as ser:
                    ser.write(command)
                    time.sleep(0.1)
                    response = ser.read(ser.in_waiting)
                    result = fr5robot.extract_and_convert(response)
                    tot+=result
            except Exception as e:
                print("Error:", str(e))
                break
        tot/=5
        a = int(tot // 100)
        b = int(tot - (a * 100))
        print(f"Received response: {a} . {b:02} g")
        if(tot>0.95*set_point and tot<1.05*set_point):
            return OK
        else:
            return OVERRANGE

    def reduce_liquild(self, set_point):
        robot=self.robot
        print("准备开始反向加料")
        OK=0
        ERROR=-1
        OVERRANGE=-2
        user_com = '/dev/ttyUSB1'
        pd=False
        plist = list(list_ports.comports())

        for port in plist:
            if '/dev/ttyUSB1' in port.device:
                pd=True
                break
        if pd==False:
            print("加液失败：未找到USB串口")
            return ERROR
        
        SPC=fr5robot.SimpleController_liquid(robot=robot, set_point=set_point)
        command = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B])
        close_protect = bytearray([0x01, 0x06, 0x00, 0x17, 0x00, 0x01, 0xF8, 0x0E])
        zero_set = bytearray([0x01, 0x06, 0x00, 0x16, 0x00, 0x01, 0xA9, 0xCE])
        open_protect = bytearray([0x01, 0x06, 0x00, 0x17, 0x00, 0x00, 0x39, 0xCE])
        try:
            with serial.Serial(user_com, 9600, timeout=1) as ser:
                print("成功连接")
                ser.write(close_protect)
                time.sleep(1)
                response = ser.read(ser.in_waiting)
                if response==close_protect:
                    print("close写保护成功")

                    ser.write(zero_set)
                    time.sleep(1)
                    response = ser.read(ser.in_waiting)
                    if response==zero_set:
                        print("零点调零成功")

                        ser.write(open_protect)
                        time.sleep(1)
                        response = ser.read(ser.in_waiting)
                        if response==open_protect:
                            print("打开写保护成功")

                            while(1):
                                ser.write(command)
                                time.sleep(0.1)
                                response = ser.read(ser.in_waiting)
                                result = fr5robot.extract_and_convert(response)*(-1)
                                # print(f"Result: {result} g")
                                if result>=SPC.set_point:
                                    print("Arrived")
                                    break
                                SPC.rd_ctr(1, result)
                                if result < 0:
                                    res = abs(result)
                                    a = int(res // 100)
                                    b = res - (a * 100)
                                    print(f"Received response: -{a} . {b:02} g")
                                else:
                                    a = int(result // 100)
                                    b = result - (a * 100)
                                    print(f"Received response: {a} . {b:02} g")
                                print("Running speed:", SPC.control_action)
                            else:
                                print("打开写保护失败")
                        else:
                            print("打开写保护失败")
                    else:
                        print("零点调零失败")
        except Exception as e:
            print("Error:", str(e))

        SPC.rd_ctr(0)
        tot=0
        for i in range(5):
            try:
                with serial.Serial(user_com, 9600, timeout=1) as ser:
                    ser.write(command)
                    time.sleep(0.1)
                    response = ser.read(ser.in_waiting)
                    result = fr5robot.extract_and_convert(response)*(-1)
                    tot+=result
            except Exception as e:
                print("Error:", str(e))
                break
        tot/=5
        a = int(tot // 100)
        b = int(tot - (a * 100))
        print(f"Received response: {a} . {b:02} g")
        if(tot>0.95*set_point and tot<1.05*set_point):
            return OK
        else:
            return OVERRANGE
        
    class SimpleController_solid:
        def __init__(self, robot, set_point, k=5.0):
            self.k = k
            self.set_point = set_point
            self.change_point = set_point/2.0
            self.robot = robot
            self.control_action = 23.0
        
        def rd_ctr(self, on, measured_value=0.0):
            if on == 0:
                self.robot.SetAO(1, 5.0, 0)
            if on == 1:
                error = self.set_point - measured_value
                if error >= self.change_point :
                    self.robot.SetAO(1, 23.0, 0)
                else:
                    self.control_action = (self.k * error) / self.change_point + 18.0
                    self.robot.SetAO(1, self.control_action, 0)
    
    def openjaw():
        OK=0
        ERROR=-1
        user_com1 = '/dev/ttyACM0'
        pd=False
        plist = list(list_ports.comports())
        for port in plist:
            print (port.device)
            if user_com1 in port.device:
                pd=True
                break
        if pd==False:
            return ERROR
        opencommand = bytearray(b'a')
        try:
            with serial.Serial(user_com1, 115200, timeout=1) as ser:
                print("成功连接ACM")
                ser.write(opencommand)
        except Exception as e:
            print("Error1:", str(e))
        time.sleep(8)
    
    def closejaw():
        OK=0
        ERROR=-1
        user_com1 = '/dev/ttyACM0'
        pd=False
        plist = list(list_ports.comports())
        for port in plist:
            print (port.device)
            if user_com1 in port.device:
                pd=True
                break
        if pd==False:
            return ERROR
        closecommand = bytearray(b'b')
        try:
            with serial.Serial(user_com1, 115200, timeout=1) as ser:
                print("成功连接ACM")
                ser.write(closecommand)
        except Exception as e:
            print("Error1:", str(e))
        time.sleep(8)

    def spin_on():
        OK=0
        ERROR=-1
        user_com1 = '/dev/ttyACM0'
        pd=False
        plist = list(list_ports.comports())
        for port in plist:
            print (port.device)
            if user_com1 in port.device:
                pd=True
                break
        if pd==False:
            return ERROR
        closecommand = bytearray(b'c')
        try:
            with serial.Serial(user_com1, 115200, timeout=1) as ser:
                print("成功连接ACM")
                ser.write(closecommand)
        except Exception as e:
            print("Error1:", str(e))
        time.sleep(8)

    def spin_off():
        OK=0
        ERROR=-1
        user_com1 = '/dev/ttyACM0'
        pd=False
        plist = list(list_ports.comports())
        for port in plist:
            print (port.device)
            if user_com1 in port.device:
                pd=True
                break
        if pd==False:
            return ERROR
        closecommand = bytearray(b'd')
        try:
            with serial.Serial(user_com1, 115200, timeout=1) as ser:
                print("成功连接ACM")
                ser.write(closecommand)
        except Exception as e:
            print("Error1:", str(e))
        time.sleep(8)
        
    def add_solid(self, set_point):
        robot = self.robot
        OK=0
        ERROR=-1
        OVERRANGE=-2
        user_com = '/dev/ttyUSB1'
        pd=False
        plist = list(list_ports.comports())
        for port in plist:
            print (port.device)
            if user_com in port.device:
                pd=True
                break
        if pd==False:
            return ERROR
        SPC=fr5robot.SimpleController_solid(robot=robot, set_point=set_point)
        command = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B])
        close_protect = bytearray([0x01, 0x06, 0x00, 0x17, 0x00, 0x01, 0xF8, 0x0E])
        zero_set = bytearray([0x01, 0x06, 0x00, 0x16, 0x00, 0x01, 0xA9, 0xCE])
        open_protect = bytearray([0x01, 0x06, 0x00, 0x17, 0x00, 0x00, 0x39, 0xCE])
        try:
            with serial.Serial(user_com, 9600, timeout=1) as ser:
                print("成功连接")
                ser.write(close_protect)
                time.sleep(1)
                response = ser.read(ser.in_waiting)
                if response==close_protect:
                    print("close写保护成功")
                    ser.write(zero_set)
                    time.sleep(1)
                    response = ser.read(ser.in_waiting)
                    if response==zero_set:
                        print("零点调零成功")

                        ser.write(open_protect)
                        time.sleep(1)
                        response = ser.read(ser.in_waiting)
                        if response==open_protect:
                            print("打开写保护成功")

                            while(1):
                                ser.write(command)
                                time.sleep(0.1)
                                response = ser.read(ser.in_waiting)
                                result = fr5robot.extract_and_convert(response)
                                if result>=SPC.set_point:
                                    print("Arrived")
                                    break
                                SPC.rd_ctr(1, result)
                                if result < 0:
                                    res = abs(result)
                                    a = int(res // 100)
                                    b = res - (a * 100)
                                    print(f"Received response: -{a} . {b:02} g")
                                else:
                                    a = int(result // 100)
                                    b = result - (a * 100)
                                    print(f"Received response: {a} . {b:02} g")
                                print("Running speed:", SPC.control_action)
                            else:
                                print("打开写保护失败")
                        else:
                            print("打开写保护失败")
                    else:
                        print("零点调零失败")
        except Exception as e:
            print("Error1:", str(e))

        SPC.rd_ctr(0)
        tot=0
        for i in range(5):
            try:
                with serial.Serial(user_com, 9600, timeout=1) as ser:
                    ser.write(command)
                    time.sleep(0.1)
                    response = ser.read(ser.in_waiting)
                    result = fr5robot.extract_and_convert(response)
                    tot += result
            except Exception as e:
                print("Error2:", str(e))
                break
        tot/=5
        a = int(tot // 100)
        b = int(tot - (a * 100))
        print(f"Received response: {a} . {b:02} g")
        if(tot>0.95*set_point and tot<1.05*set_point):
            return OK
        else:
            return OVERRANGE