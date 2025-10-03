import serial
import os
import sys
import struct
import keyboard
 
COMMAND_OFFSET_CMD		    =   0
COMMAND_OFFSET_DIRECTION    =   1
COMMAND_OFFSET_SPEED        =   2
COMMAND_OFFSET_ANGLE        =   2

COMMAND_CMD_STOP            =   0
COMMAND_CMD_SET_SPEED       =   1
COMMAND_CMD_SET_ANGLE       =   2

COMMAND_DIRECTION_FRONT     =   0
COMMAND_DIRECTION_BACK      =   1
COMMAND_DIRECTION_LEFT      =   2
COMMAND_DIRECTION_RIGHT     =   3

class CarControl:
    port = ""
    bandrate = 9600
    def __init__(self,port,bandrate=9600):
        self.port = port
        self.bandrate = bandrate
        self.ser = serial.Serial(port=port,baudrate=bandrate)
        self.SetSpeed(0)
        self.SetAngle(0)
        self.SetSpeed(0)
        self.SetAngle(0)
        self.ser.read_all()
        
    def SetSpeed(self,speed:int):
        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100
        
        if speed >= 0:
            data = struct.pack(">BBB",COMMAND_CMD_SET_SPEED,COMMAND_DIRECTION_FRONT,speed) + b"\xcc\xcc"
        else:
            data = struct.pack(">BBB",COMMAND_CMD_SET_SPEED,COMMAND_DIRECTION_BACK,-speed) + b"\xcc\xcc"
        print("[*]","设置速度",speed,"%")
        self.ser.write(data)
        
    def SetAngle(self,angle:int):
        if angle > 30:
            angle = 30
        elif angle < -30:
            angle = -30
        
        if angle >= 0:
            data = struct.pack(">BBB",COMMAND_CMD_SET_ANGLE,COMMAND_DIRECTION_RIGHT,angle) + b"\xcc\xcc"
        else:
            data = struct.pack(">BBB",COMMAND_CMD_SET_ANGLE,COMMAND_DIRECTION_LEFT,-angle) + b"\xcc\xcc"
        print("[*]","设置转向角度",angle,"%")
        self.ser.write(data)
        
    def Stop(self):
        print("[*]","停止")
        data = struct.pack(">B",COMMAND_CMD_STOP) + b"\xcc\xcc"
        self.ser.write(data)

directions = {
    'up': 'up',
    'down': 'down',
    'left': 'left',
    'right': 'right',
    'space': 'space',
}

cc = None

is_pressed = {"up":False,"down":False,"left":False,"right":False}

def on_direction(event):
    direction = directions.get(event.name)
    if event.event_type == "down":
        if direction == "up":
            if is_pressed[direction] == False:
                cc.SetSpeed(100)
                is_pressed[direction] = True
        elif direction == "down":
            if is_pressed[direction] == False:
                cc.SetSpeed(-100)
                is_pressed[direction] = True
        elif direction == "left":
            if is_pressed[direction] == False:
                cc.SetAngle(-30)
                is_pressed[direction] = True
        elif direction == "right":
            if is_pressed[direction] == False:
                cc.SetAngle(30)
                is_pressed[direction] = True
        elif direction == "space":
            if is_pressed[direction] == False:
                cc.Stop()
                is_pressed[direction] = True
    else:
        if direction == "up":
            cc.SetSpeed(0)
            is_pressed[direction] = False
        elif direction == "down":
            cc.SetSpeed(0)
            is_pressed[direction] = False
        elif direction == "left":
            cc.SetAngle(0)
            is_pressed[direction] = False
        elif direction == "right":
            cc.SetAngle(0)
            is_pressed[direction] = False
        elif direction == "space":
            cc.Stop()


def main():
    global cc
    if len(sys.argv) != 2:
        print("[*]","请提供串口号")
        sys.exit(-1)
    com = sys.argv[1]
    print("[+]","串口号:",com)
    cc = CarControl(com)
    keyboard.hook(on_direction)
    keyboard.wait('q')
    # input()
    # cc.SetSpeed(-100)
    # input()
    # cc.SetAngle(-30)
    # input()
    # cc.SetAngle(30)
    # input()
    # cc.SetAngle(0)
    # input()
    # cc.Stop()
    # 方向键的映射

 
# 开始监听

    
if __name__ == "__main__":
    main()
    
    
    
    