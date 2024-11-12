# coding=utf-8

import Robot
import struct
from serial.tools import list_ports
import serial
import time
class Add_liquid:
    class SimpleController:
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

    def read():
        OK=0
        ERROR=-1
        OVERRANGE=-2
        user_com = '/dev/ttyUSB0'
        pd=False
        plist = list(list_ports.comports())
        for port in plist:
            print (port.device)
            if '/dev/ttyUSB0' in port.device:
                pd=True
                break
        if pd==False:
            return ERROR
        command = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B])
        try:
            with serial.Serial(user_com, 9600, timeout=1) as ser:
                print("成功连接")
                ser.write(command)
                time.sleep(0.1)
                response = ser.read(ser.in_waiting)
                result = Add_liquid.extract_and_convert(response)
                print(result)

        except Exception as e:
            print("Error1:", str(e))

    def extract_and_convert(hex_data):
        # 确保传入的数据是bytes类型
        if not isinstance(hex_data, bytes) or len(hex_data) != 9:
            raise ValueError("Invalid bytes data provided")

        bytes_data = hex_data[5:7]
        bytes_data += hex_data[3:5]
        
        # 使用struct解包为有符号整数
        result = struct.unpack('>i', bytes_data)[0]

        return result

    def add_liquild(robot, set_point):
        OK=0
        ERROR=-1
        OVERRANGE=-2
        user_com = '/dev/ttyUSB0'
        pd=False
        plist = list(list_ports.comports())
        for port in plist:
            print (port.device)
            if '/dev/ttyUSB0' in port.device:
                pd=True
                break
        if pd==False:
            return ERROR
        SPC=Add_liquid.SimpleController(robot=robot, set_point=set_point)
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
                                result = Add_liquid.extract_and_convert(response)
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
        robot.SetAO(0, -100.0, 0)
        print(111)
        time.sleep(10)
        robot.SetAO(0, 0.0, 0)
        tot=0
        for i in range(5):
            try:
                with serial.Serial(user_com, 9600, timeout=1) as ser:
                    ser.write(command)
                    time.sleep(0.1)
                    response = ser.read(ser.in_waiting)
                    result = Add_liquid.extract_and_convert(response)
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

if __name__ == "__main__":
    robot = Robot.RPC('192.168.59.6')
    # Add_liquid.read()
    Add_liquid.add_liquild(robot, 1000.0)
