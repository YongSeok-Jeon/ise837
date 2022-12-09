import sys
import time
from MachineMotion import *
from tkinter import *
from tkinter import ttk
import numpy as np
import requests
from matplotlib import pyplot as plt
from PIL import ImageTk, Image
import io
import cv2
import time
import rtde_receive
import rtde_control
import requests
from matplotlib import pyplot as plt
from PIL import ImageTk, Image
from robotiq_preamble import ROBOTIQ_PREAMBLE
import time

#####################################
### Gripper control
#####################################
class RobotiqGripper(object):
    def __init__(self, rtde_c): 
        
        self.rtde_c = rtde_c

    def call(self, script_name, script_function):
        return self.rtde_c.sendCustomScriptFunction(
            "ROBOTIQ_" + script_name,
            ROBOTIQ_PREAMBLE + script_function)

    def activate(self):
        ret = self.call("ACTIVATE", "rq_activate()")
        time.sleep(5)  # HACK
        return ret

    def set_speed(self, speed): # speed (int): speed as a percentage [0-100]
        return self.call("SET_SPEED", "rq_set_speed_norm(" + str(speed) + ")")

    def set_force(self, force):# force (int): force as a percentage [0-100]
        return self.call("SET_FORCE", "rq_set_force_norm(" + str(force) + ")")

    def move(self, pos_in_mm):
        return self.call("MOVE", "rq_move_and_wait_mm(" + str(pos_in_mm) + ")")

    def open(self):
        return self.call("OPEN", "rq_open_and_wait()")

    def close(self):
        return self.call("CLOSE", "rq_close_and_wait()")

#####################################
### Conveyor control
#####################################


class ConveryorControl:
    def __init__(self, ip):
        self.controller = MachineMotion(machineIp=ip)
        self.speed = 50
        self.acceleration = 50
        self.controller.setSpeed(self.speed)
        self.controller.setAcceleration(self.acceleration)

    def set_acceleration(self, acceleration, axis=1):
        self.acceleration = acceleration
        self.controller.setAcceleration(acceleration)
    
    def set_velocity(self, velocity, axis=1):
        self.speed = velocity
        self.controller.setSpeed(velocity)

    def move_continuous(self, direction, axis=1):
        self.controller.moveContinuous(axis, self.speed * direction, self.acceleration)
    
    def stop_continuous(self, axis=1):
        self.controller.stopMoveContinuous(axis, self.acceleration)
        
    def sensor(self, axis=1):
        self.sensor_value = self.controller.digitalRead(1,3) #(networkid,pin)
        return self.sensor_value
        
#####################################
### Variable
#####################################
how_long = 180 #sec

host = "192.168.10.83"

TCP_starting_point = [90, -45, 90, -135, 45, 0]
TCP_conveyor_camera = [32.77, 16.90, -92.62, -99.59, 64.79, -183.88]
TCP_conveyor_grip_1 = [50.12, 8.68, -128.19, -6.16, 58.11, -221.78]
TCP_conveyor_grip_2 = [53.07, -2.07, -126.47, 8.98, 55.37, -221.50]

rtde_r = rtde_receive.RTDEReceiveInterface(host)
rtde_c = rtde_control.RTDEControlInterface(host)

time.sleep(1)
gripper = RobotiqGripper(rtde_c)
gripper.activate()
gripper.open()
time.sleep(3)
#
# rtde_c.moveJ([a*3.1415/180 for a in TCP_conveyor_camera])
# 

#####################################
### Activate function
#####################################
start_time = time.time()

aa = 1
cc = 1
status = 0

cc1 = ConveryorControl("192.168.10.122")
cc1.set_acceleration(30)
cc1.set_velocity(30)
cc1.move_continuous(1)

while (aa != 0):
    
    ###########
    ### Move conveyor
    ###########
    while (cc != 0):
        cc1.move_continuous(1)
        temp = []
        
        if cc1.sensor() == 1: # detect object
            cc1.stop_continuous()
            rtde_c.moveJ([a*3.1415/180 for a in TCP_conveyor_camera])
            time.sleep(2)
            cc = 0
            bb = 1 # Activate camera
    
    ##########
    ### Camera detect 
    ##########
    
    while (bb != 0):
        
        img = np.zeros((480,640,3))
        resp = requests.get("http://"+ "192.168.10.83"+":4242/current.jpg?type=color", stream=True).raw
        img = np.asarray(bytearray(resp.read()), dtype="uint8")
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        lb_red = np.array([150, 50, 70], "uint8")
        ub_red = np.array([200, 255, 255], "uint8")
        red_mask = cv2.inRange(hsv, lb_red, ub_red)

        lb_green = np.array([40, 150, 50], "uint8")
        ub_green = np.array([70, 255, 100], "uint8")
        green_mask = cv2.inRange(hsv, lb_green, ub_green)

        lb_blue = np.array([100, 200, 50], "uint8")
        ub_blue = np.array([130, 255, 255], "uint8")
        blue_mask = cv2.inRange(hsv, lb_blue, ub_blue)

        lb_yellow = np.array([0, 100, 100], "uint8")
        ub_yellow = np.array([50, 255, 255], "uint8")
        yellow_mask = cv2.inRange(hsv, lb_yellow, ub_yellow)

        kernel = np.ones((5, 5), "uint8")

        red_mask = cv2.dilate(red_mask, kernel)
        green_mask = cv2.dilate(green_mask, kernel)
        blue_mask = cv2.dilate(blue_mask, kernel)
        yellow_mask = cv2.dilate(yellow_mask, kernel)

        ########################
        ### RGB/Area of interests
        ########################
        min_area_red = 5000
        max_area_red = 20000

        min_area_green =  5000
        max_area_green =  20000

        min_area_blue =  3000
        max_area_blue =   20000

        min_area_yellow =  5000
        max_area_yellow =   20000

        area_x_min = 200
        area_x_max = 350
        area_y_min = 150
        area_y_max = 500

        ########################
        ### Red
        ########################
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (min_area_red <= area and max_area_red >= area):
                x,y,w,h = cv2.boundingRect(contour)
                if (area_x_min <= x and x <= area_x_max):
                    if (area_y_min <= y and y <= area_y_max):
                        if w <= 80:
                            temp.append("red_1x3")
                        if w > 95:
                            temp.append("red_2x2")
                            

        ########################
        ### Green
        ########################
        contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (min_area_green <= area and max_area_green >= area):
                x,y,w,h = cv2.boundingRect(contour)
                if (area_x_min <= x and x <= area_x_max):
                    if (area_y_min <= y and y <= area_y_max):
                        if h <= 180:
                            temp.append("green_1x2")
                        if h > 180:
                            temp.append("green_1x4")

        ########################
        ### Blue
        ########################
        contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (min_area_blue <= area and max_area_blue >= area):
                x,y,w,h = cv2.boundingRect(contour)
                if (area_x_min <= x and x <= area_x_max):
                    if (area_y_min <= y and y <= area_y_max):
                        if h <= 100:
                            temp.append("blue_1x1")
                        if h > 100:
                            temp.append("blue_1x4")

        ########################
        ### Yellow
        ########################
        contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area >= min_area_yellow and area <= max_area_yellow:
                x,y,w,h = cv2.boundingRect(contour)
                if (area_x_min <= x and x <= area_x_max):
                    if (area_y_min <= y and y <= area_y_max):
                        temp.append("yellow_2x2")
        
        if len(temp) >= 10:
            temp_color, temp_number = np.unique(np.array(temp), return_counts = True)
            if np.max(temp_number) >= 5:
                status = temp_color[np.argmax(temp_number)]
                bb = 0
                        
    if status in ["red_1x3", "red_2x2", "green_1x2", "green_1x4", "blue_1x1", "blue_1x4", "yellow_2x2"]:
        print(status)
        rtde_c.moveJ([a*3.1415/180 for a in TCP_conveyor_grip_1])
        time.sleep(0.5)
        rtde_c.moveJ([a*3.1415/180 for a in TCP_conveyor_grip_2])
        time.sleep(0.5)
        
        
        gripper.close()
        rtde_c.reconnect()
        time.sleep(1)
        
        rtde_c.moveJ([a*3.1415/180 for a in TCP_conveyor_grip_1])
        time.sleep(0.5)
        rtde_c.moveJ([a*3.1415/180 for a in TCP_conveyor_camera])
        time.sleep(1)
        rtde_c.moveJ([a*3.1415/180 for a in TCP_starting_point])
        
        
        gripper.open()
        rtde_c.reconnect()
        time.sleep(1)
        
        status = 0
        cc = 1

    end_time = time.time()
    if end_time - start_time >= how_long:
        cv2.destroyAllWindows()
        cc = 0
        bb = 0
        aa = 0
        cc1.stop_continuous()