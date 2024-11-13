'''
法奥新版SDK，适配fr5机械臂，python=3.10
update date 2024-04-11
'''

import sys
import os
# 获取当前文件的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 将包含 Robot.so 的目录添加到 sys.path
sys.path.append(current_dir)
import copy
import time
import Robot
import numpy as np
import rospy
from std_msgs.msg import Int32
import threading


def invoke_api(api_lambda, block=True, error_msg='failed to invoke api'):
    """
        调用API接口

        api_lambda 使用举例：
        api_lambda = lambda: self.robot.GetActualToolFlangePose(0)

        api_lambda: API接口
        block: 是否阻塞
        error_msg: 错误信息
    """
    raw_data = api_lambda()
    # raw_data is a tuple, [ret, data], if no error
    while (raw_data is not tuple or raw_data[0] != 0) and block:
        # The sdk returns an error code
        print(f'ret: {raw_data[0] if raw_data is tuple else raw_data}. {error_msg}')
        raw_data = api_lambda()
        time.sleep(0.25)
    if raw_data is tuple and raw_data.length == 2:
        return raw_data[1]  # 返回数据
    else:
        if raw_data is tuple:
            raw_data = raw_data[0]  # 返回错误码
        print(error_msg)
        return raw_data


class fr5robot:
    def __init__(self, index=1):
        # 初始化ROS节点
        rospy.init_node('fr5_main', anonymous=True)

        # 初始化变量
        if index == 1:
            self.robot = Robot.RPC('192.168.59.6')
        elif index == 2:
            self.robot = Robot.RPC('192.168.58.6')
        self.index = index

        # 夹爪初始化
        self.robot.SetGripperConfig(4, 0, 0, 1)
        time.sleep(0.5)
        self.robot.ActGripper(1, 1)
        time.sleep(2)
        self.robot.MoveGripper(1, 100, 50, 10, 10000, 1)
        time.sleep(0.5)

        print("夹爪初始化完成")

        # 初始化ROS发布者
        self.pub_gripper1 = rospy.Publisher('pub_gripper1', Int32, queue_size=0)
        self.pub_gripper2 = rospy.Publisher('pub_gripper2', Int32, queue_size=0)

    def MoveGripper(self, index, pos, speed, force, maxtime, block):
        '''
        控制夹爪移动
        index: 夹爪编号
        pos: 位置百分比，范围[0~100]
        speed: 速度百分比，范围[0~100]
        force: 力矩百分比，范围[0~100]
        maxtime: 最大等待时间，范围[0~30000]，单位[ms]
        block: 0-阻塞，1-非阻塞
        '''
        self.robot.MoveGripper(index, pos, speed, force, maxtime, block)
        gripper_pos = Int32()
        gripper_pos.data = pos
        if self.index == 1:
            self.pub_gripper2.publish(gripper_pos)
        elif self.index == 2:
            self.pub_gripper1.publish(gripper_pos)
        else:
            exit()

    def Go_to_start_zone(self, v=30.0, open=1):
        """
        机械臂复位
        """
        self.point_safe_move([0.0, -250.0, 400.0, 90.0, 0.0, 0.0], v, 200.0)
        if open:
            self.MoveGripper(1, 100, 50, 10, 10000, 1)

    def dou_go_start(self, fr5_B, v=50.0):
        """
        两个机械臂同时复位
        """
        self.Go_to_start_zone(v)
        time.sleep(1)
        fr5_B.Go_to_start_zone(v)

    def MoveL(self, x=0.000, y=0.000, z=0.000, movespeed=100.0):
        """
        机械臂直线运动
        """
        eP1 = [0.000, 0.000, 0.000, 0.000]
        dP1 = [x, y, z, 0.000, 0.000, 0.000]
        pos_now = self.GetActualToolFlangePose()

        # 更新位置
        pos_now[0] = x
        pos_now[1] = y
        pos_now[2] = z
        ret = self.robot.MoveL(pos_now, 0, 0, blendR=-1.0)
        print('error_code', ret)

    def MoveLDelta(self, x=0.000, y=0.000, z=0.000, movespeed=100.0):
        """
        机械臂直线运动
        """
        eP1 = [0.000, 0.000, 0.000, 0.000]
        dP1 = [x, y, z, 0.000, 0.000, 0.000]
        pos_now = self.GetActualToolFlangePose()

        # 更新位置
        pos_now[0] += x
        pos_now[1] += y
        pos_now[2] += z
        ret = self.robot.MoveL(pos_now, 0, 0, blendR=-1.0)
        print('error_code', ret)

    def Safe_move(self, start_catch_position, dir, v=30.0):
        """
        机械臂安全移动到指定位置
        start_catch_position: 起始位置
        dir: 方向
        v: 速度
        """
        temp_path = []
        start_interpolation_path = []

        # 获取机械臂当前末端位置
        start_position = self.GetActualToolFlangePose()
        
        # 保存初始路径点
        start_interpolation_path += [start_position]

        # 添加路径点：将Z轴移动到250.0，保持其他坐标不变
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
            # 添加路径点：沿Y轴移动到目标位置，保持Z轴高度为250.0
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

            # 添加路径点：沿Z轴下降到目标位置
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

            # 添加路径点：沿Y轴返回100.0
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
            # 添加路径点：沿X轴移动到目标位置，保持Z轴高度为250.0
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

            # 添加路径点：沿Z轴下降到目标位置
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

            # 添加路径点：沿X轴返回100.0
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

        # 打印路径点信息
        print(start_interpolation_path)

        # 执行移动
        for i in range(len(start_interpolation_path)):
            self.robot.MoveCart(
                start_interpolation_path[i], 0, 0, 0.0, 0.0, v, -1.0, -1
            )

    def point_safe_move(self, start_catch_position, v=60.0, height=250.0, last_v=0):
        '''
        机械臂安全运动到指定位置
        '''
        end_height = self.GetActualToolFlangePose()

        if end_height[2] < height:
            self.MoveLDelta(0.0, 0.0, max(height, start_catch_position[2]) - end_height[2], v)
        time.sleep(1)

        middle_pos = copy.deepcopy(start_catch_position)
        end_height = self.GetActualToolFlangePose(error_msg='failed to get end_height_from_sdk2')
        middle_pos[2] = max(end_height[2], start_catch_position[2], height)
        self.robot.MoveCart(middle_pos, 0, 0)

        time.sleep(1)

        end_height = self.GetActualToolFlangePose(error_msg='failed to get end_height_from_sdk3')
        if last_v == 0:
            last_v = v
        self.MoveL(0.0, 0.0, (start_catch_position[2] - end_height[2]), last_v)

    def pour(self, r, h, pour_position, pour_direction, sel_num, i=-2, max_angel=90, rate=100.0, v=70.0, upright=1, shake=1):
        '''
        倾倒操作
        r: 容器半径
        h: 容器上平面离夹爪中心高度
        pour_position: 倾倒位置
        pour_direction: 倾倒方向
        sel_num: 选择的容器，1---试管 2---烧杯 3---量筒 4---反应瓶 5---引导器 6---漏斗 7--蒸发皿
        i: 角度增量及方向
        max_angel: 最大旋转角度
        rate: 指令周期
        v: 速度
        upright: 是否需要上抬以规避倾倒仪器回归水平位时的碰撞
        shake: 是否需要摇晃
        '''
        real_bias = b_bias
        rxryrz = []
        if pour_direction == "yn":
            pour_position[1] += real_bias
            rxryrz = [90.0, 0.0, 0.0]
            dir = "yn"
        elif pour_direction == "xn":
            pour_position[0] += real_bias
            rxryrz = [90.0, 0.0, -90.0]
            dir = "xn"
        elif pour_direction == "xp":
            pour_position[0] -= real_bias
            rxryrz = [90.0, 0.0, 90.0]
            dir = "xp"
        elif pour_direction == "yp":
            pour_position[1] -= real_bias
            rxryrz = [90.0, 0.0, -179.9]
            dir = "yp"
        else:
            print("------error!-------")
            exit()

        if len(pour_position) != 2:
            print("xy坐标错误")
            exit()

        while True:
            if int(sel_num) == 1:
                pour_position += [130.0]    # 此处数据不全部准确，需要实地测量
                pour_position += rxryrz
                break
            elif int(sel_num) == 2:
                pour_position += [130.0]
                pour_position += rxryrz
                break
            elif int(sel_num) == 3:
                pour_position += [65.0]
                pour_position += rxryrz
                break
            elif int(sel_num) == 4:
                pour_position += [168.0]
                pour_position += rxryrz
                break
            elif int(sel_num) == 5:
                pour_position += [150.0]
                pour_position += rxryrz
                break
            elif int(sel_num) == 6:
                pour_position += [400.0]    
                pour_position += rxryrz
                break
            elif int(sel_num) == 7:
                pour_position += [450.0]    
                pour_position += rxryrz
                break
            else:
                print("--------输入错误！请重新输入！---------")
                continue

        if pour_direction == "yn":
            position1 = pour_position
            position1[1] += 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(0.0, -100.0, 0.0)
            time.sleep(1)
        elif pour_direction == "xn":
            position1 = pour_position
            position1[0] += 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(-100, 0.0, 0.0)
            time.sleep(1)
        elif pour_direction == "xp":
            position1 = pour_position
            position1[0] -= 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(100, 0.0, 0.0)
            time.sleep(1)
        elif pour_direction == "yp":
            position1 = pour_position
            position1[1] -= 100
            position1[2] += 200
            self.robot.MoveCart(position1, 0, 0)
            time.sleep(2)
            self.MoveLDelta(0.0, 0.0, -200.0)
            time.sleep(1)
            self.MoveLDelta(0.0, 100.0, 0.0)
            time.sleep(1)
            
        rate /= 100
        types = {
            "100ml": {"diameter": 10, "height": 20},
            "200ml": {"diameter": 15, "height": 25},
            "250ml": {"diameter": 12, "height": 22},
        }
        
        # 伺服运动参数预置
        t = 0.002
        eP0 = [0.000, 0.000, 0.000, 0.000]
        dP0 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
        gain = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # 位姿增量比例系数，仅在增量运动下生效，范围[0~1]

        P1 = self.GetActualToolFlangePose(error_msg='failed to get P1 from sdk')
        J1 = invoke_api(lambda: self.robot.GetActualJointPosDegree(0), error_msg='failed to get J1 from sdk')

        # 计算旋转参数
        R = np.sqrt(r ** 2 + h ** 2)
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

            joint_pos = invoke_api(lambda :self.robot.GetActualJointPosDegree(), error_msg="when executing pouring,failed to get end_height_from_sdk2")
            joint_pos[5] += i * rate

            self.robot.ServoJ(joint_pos, 0.0, 0.0, t, 0.0, 0.0)  # 关节角增量移动

            time.sleep(t)

            # 更新差值
            joint_pos_difference = joint_pos[5] - J1[5]

        joint_pos = invoke_api(lambda: self.robot.GetActualJointPosDegree(0), error_msg='failed to get joint_pos from sdk')
        pos_record = invoke_api(lambda: self.robot.GetActualTCPPose(0), error_msg='failed to get pos record from sdk')
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

    def GetActualToolFlangePose(self, block=True, error_msg='failed to get pose'):
        '''
            获取机械臂末端位置，也可用于判断动作是否完成
        '''
        return invoke_api(lambda: self.robot.GetActualToolFlangePose(0), block, error_msg)