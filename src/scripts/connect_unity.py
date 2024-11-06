#!/bin/python3
# -*- coding: utf-8 -*-
import rospy
# from std_msgs.msg import String,Int32
import socket
import sys
import struct
# from moveit_msgs.msg import joint_pos
from unity_robotics_demo_msgs.msg import Connect_unity_test1
# import force_sensor
# from std_msgs.msg import UInt8MultiArray,Float32MultiArray,String
gripper1_pos = 100
gripper2_pos = 100
# 力传感器数据，分别为xyz的力（单位：N），xyz的力矩（单位：Nm）
force_data = [0.0,0.0,0.0,0.0,0.0,0.0]


def gripper1(pos):
    global gripper1_pos
    gripper1_pos = pos.data

def gripper2(pos):
    global gripper2_pos
    gripper2_pos = pos.data
    
def force_callback(force_msg):
    global force_data 
    force_data= force_msg.data
    # print(force_data)
    
    # force_data = list(force_data)

def talker():
    pub = rospy.Publisher('Connect_unity_test', Connect_unity_test1, queue_size=10)
    rospy.init_node('joint_publisher', anonymous=True)
    # rospy.Subscriber('pub_gripper1',Int32,gripper1)
    # rospy.Subscriber('pub_gripper2',Int32,gripper2)
    # rospy.Subscriber('/force_msg',Float32MultiArray,force_callback)
    print('=============')
    Con_SOCKETINFO1 = socket.socket()
    errorinfo = Con_SOCKETINFO1.connect_ex(('192.168.58.6', 8083))
    if errorinfo == 0:
        print("机器人1 8083端口连接成功")
    else:
        print("机器人1 8083端口连接失败,退出程序")
        Con_SOCKETINFO1.close()
        sys.exit()

    Con_SOCKETINFO2 = socket.socket()
    errorinfo = Con_SOCKETINFO2.connect_ex(('192.168.59.6', 8083))
    if errorinfo == 0:
        print("机器人2 8083端口连接成功")
    else:
        print("机器人2 8083端口连接失败,退出程序")
        Con_SOCKETINFO2.close()
        sys.exit()
    
    last_arm1 = [0,0,0,0,0,0]
    last_arm2 = [0,0,0,0,0,0]

    while not rospy.is_shutdown():
        RV1 = Con_SOCKETINFO1.recv(1024)
        POSE_name = ["J1", "J2", "J3", "J4", "J5", "J6", "X_POSE", "Y_POSE", "Z_POSE", "RX_POSE", 'RY_POSE', "RZ_POSE"]
        infosend_arm1 = ["", "", "", "", "", "", "", "", "", "", '', "",'']
        for i in range(len(POSE_name)):
            info_arm1 = str(struct.unpack("d", RV1[(i + 1) * 8:(i + 2) * 8]))
            info_arm1 = info_arm1.strip("(").strip(")").strip(",")
            infosend_arm1[i] = info_arm1 
        infosend_arm1[len(POSE_name)] = str(int.from_bytes(RV1[234:237], byteorder="little"))

        RV2 = Con_SOCKETINFO2.recv(1024)
        POSE_name = ["J1", "J2", "J3", "J4", "J5", "J6", "X_POSE", "Y_POSE", "Z_POSE", "RX_POSE", 'RY_POSE', "RZ_POSE"]
        infosend_arm2 = ["", "", "", "", "", "", "", "", "", "", '', "",'']
        for i in range(len(POSE_name)):
            info_arm2 = str(struct.unpack("d", RV2[(i + 1) * 8:(i + 2) * 8]))
            info_arm2 = info_arm2.strip("(").strip(")").strip(",")
            infosend_arm2[i] = info_arm2 
        infosend_arm2[len(POSE_name)] = str(int.from_bytes(RV2[234:237], byteorder="little"))
            
        pub_string = Connect_unity_test1()


        for i in range(6):
            pub_string.arm1[i] = float(infosend_arm1[i])
            # pub_string.arm1_vel[i] = (pub_string.arm1[i] - last_arm1[i])/0.04
            pub_string.arm2[i] = float(infosend_arm2[i])
            # pub_string.arm2_vel[i] = (pub_string.arm2[i] - last_arm2[i])/0.04
            # pub_string.arm1_tor[i] = float(str(struct.unpack("d", RV1[i * 8 + 108:(i + 1) * 8 + 108])).strip("(,)"))
            # pub_string.arm2_tor[i] = float(str(struct.unpack("d", RV2[i * 8 + 108:(i + 1) * 8 + 108])).strip("(,)"))

        for i in range(6):
            last_arm1[i] = pub_string.arm1[i]
            last_arm2[i] = pub_string.arm2[i] 
        pub_string.arm1_catcher = gripper1_pos
        pub_string.arm2_catcher = gripper2_pos
        
        # for i in range(6):
        #     pub_string.force_data[i] = force_data[i]

        
        pub.publish(pub_string)
        
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass






