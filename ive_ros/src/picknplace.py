#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from pymycobot.mycobot import MyCobot
import time
import cv2
import os
from pyzbar import pyzbar
import socket


# 그리퍼 init
def init_gripper(mc):
    
    # print("go to home", mc.get_angles())
    # mc.send_angles([0, 0, 0, 0, 0, 0], 15)
    # time.sleep(10)
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()
    # time.sleep(1)

# 로봇팔 home position
def init_position(mc):
    print("go to home")
    mc.send_angles([0,0,0,0,0,0], 100)
    time.sleep(2)

# plc에 신호 보내기
def send_to_plc(mc):
    # plc에 동작 신호 보내기
    mc.set_basic_output(1,0)
    # time.sleep(1)
    mc.set_basic_output(1,1)
    # time.sleep(1)
    mc.set_basic_output(2,1)
    # time.sleep(1)

# 스캔 위치로 이동
def goto_photo(mc):
    
    # mc.send_angles([90, 0, 0, 0, -90, 0], 90)
    # time.sleep(5)  # 로봇 각 동작 사이의 딜레이 설정
    # mc.send_coords([-100, -250, 350, -89.99, 90, -179.91], 90)
    # coords = mc.get_coords()
    # print(coords)
    # mc.send_coords([119.5, -200.5, 195, -99.97, -5.63, -99.04], 15)
    # time.sleep(5)
    print("go to take photo")
    mc.send_coords([142.1, 4.5, 320, -180, 0.0, -90], 100)
    time.sleep(5)
    print(mc.get_coords())

# 피킹 위치로 이동
def goto_pick(mc, x, y, ang):
    print("go to pick place", mc.get_angles())
    mc.send_coords([235 - y, -5 - x, 170, 180, 0, -ang], 100)
    print(f"@@@@@@@: {mc.get_coords()}")
    time.sleep(2.5)



# 피킹 직전 위치까지 이동
def picking_control(mc, x, y, ang):
    mc.send_coords([235 - y, -5 - x, 140, 180, 0, -ang], 60)
    time.sleep(4)
    print("on the picking area", mc.get_angles())


# 플레이싱 위치로 이동
def goto_place(mc, x_offset, z_offset):
    # 플레이싱 진입 모션
    mc.send_coords([0 + x_offset, -230 , 150, -180, -0, -90], 100)
    time.sleep(2)
    print("go to place", mc.get_angles())
    # 플레이싱 포지션
    mc.send_coords([0 + x_offset, -230 , 25 + z_offset, -180, -0, -90], 70)
    time.sleep(5)

# 그리퍼 열기
def open_gripper(mc):
    print("open gripper")
    mc.set_eletric_gripper(0)
    mc.set_gripper_value(100, 80, 1)
    # print(mc.get_gripper_value())
    time.sleep(1)
    # print(mc.get_gripper_value())

# 그리퍼 50만큼만 열기
def mid_gripper(mc):
    mc.set_eletric_gripper(1)
    mc.set_gripper_value(50, 20, 1)
    time.sleep(1)

# 그리퍼 닫기
def close_gripper(mc):
    print("close gripper")
    mc.set_eletric_gripper(1)
    mc.set_gripper_value(16, 80, 1)
    # print(mc.get_gripper_value())
    time.sleep(1)
    # print(mc.get_gripper_value())
    
    
