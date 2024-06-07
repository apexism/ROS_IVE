# src/pick_place.py

from pymycobot.mycobot import MyCobot
import time
import cv2
import os
from pyzbar import pyzbar
import socket


def goto_angles(mc, angles):
    mc.send_angles(angles,20)
    i=0
    while True:
        if mc.is_in_position(angles,0) == True:
            i+=1
            if i == 12:
                break
    print("도착: ", mc.get_angles())

def goto_coords(mc, angles):
    mc.send_coords(angles,50,1)
    i=0
    while True:
        if mc.is_in_position(angles,1) == True:
            i+=1
            if i == 12:
                break
    print("도착: ", mc.get_coords())
        
def open_gripper(mc):
    mc.set_eletric_gripper(0)
    mc.set_gripper_value(80, 80, 1)
    while True:
        if mc.is_gripper_moving() == 1:
            print(mc.get_gripper_value(1))
        elif mc.is_gripper_moving() == 0:
            print(mc.get_gripper_value(1))
            break

def mid_gripper(mc):
    mc.set_gripper_value(18, 80, 1)
    while True:
        if mc.is_gripper_moving() == 1:
            print(mc.get_gripper_value(1))
        elif mc.is_gripper_moving() == 0:
            print(mc.get_gripper_value(1))
            break
def close_gripper(mc):
    mc.set_gripper_value(0, 80, 1)
    while True:
        if mc.is_gripper_moving() == 1:
            print(mc.get_gripper_value(1))
        elif mc.is_gripper_moving() == 0:
            print(mc.get_gripper_value(1))
            break
