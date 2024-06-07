#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import os

# 절대 경로를 PYTHONPATH에 추가
sys.path.append('/home/jwy/catkin_ws/src/ive_ros/src')

import time


import picknplace as pp
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from pynput import keyboard

import cv2 as cv
from pyzbar import pyzbar
import time
from cv2 import aruco
import numpy as np

left_x = None
left_y = None
right_x = None
right_y = None
cent_x = None
cent_y = None
status = "updown_OK"


def scan_qr_codes(image):
    """이미지에서 QR 코드를 스캔하고 검출된 QR 코드의 데이터를 반환합니다."""
    qr_codes = pyzbar.decode(image)
    return [qr_code.data.decode('utf-8') for qr_code in qr_codes]

def scan_frame(frame, height, width, scan_size, step_right, step_down, show=False):
    """ 주어진 이미지 프레임을 주기적으로 스캔하여 QR 코드를 탐색하고 위치를 반환합니다."""
    detected_qr_codes = set()
    qr_code_locations = []

    y = 0
    while y + scan_size[1] <= height:
        x = 0
        while x + scan_size[0] <= width:
            roi = frame[y:y+scan_size[1], x:x+scan_size[0]]
            qr_codes = scan_qr_codes(roi)
            for code in qr_codes:
                if code not in detected_qr_codes:
                    detected_qr_codes.add(code)
                    qr_code_locations.append((code, (x, y)))
            if show:
                display = cv.rectangle(frame.copy(), (x, y), (x + scan_size[0], y + scan_size[1]), (0, 255, 0), 2)
                cv.imshow("Scanning QR Codes", display)
                cv.waitKey(1)  # Refresh display
            x += step_right
        y += step_down
    return detected_qr_codes, qr_code_locations

def aruco_scan(frame, calib_data):
    global left_x
    global left_y
    global right_x
    global right_y
    global cent_x
    global cent_y
    """아르코 마커를 스캔하고 각 마커의 위치와 아이디 정보를 화면에 표시합니다."""
    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]

    MARKER_SIZE = 0.02 # meters
    # marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    # # param_markers = cv.aruco.DetectorParameters_create()
    # param_markers = cv.aruco.DetectorParameters()
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    param_markers = aruco.DetectorParameters_create()



    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = cv.aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_corners:
        rVec, tVec, _ = cv.aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        for ids, corners, rvec, tvec in zip(marker_IDs, marker_corners, rVec, tVec):
            cv.aruco.drawDetectedMarkers(frame, [corners])
            cv.aruco.drawAxis(frame, cam_mat, dist_coef, rvec, tvec, 0.02)
            
            if ids %2 == 0:
                left_x = corners[0][0][0]
                left_y = corners[0][0][1]
            elif ids %2 == 1:
                right_x = corners[0][0][0]
                right_y = corners[0][0][1]
            elif ids == None:
                left_x = None
                left_y = None
                right_x = None
                right_y = None
            if left_x is not None and left_y is not None and right_x is not None and right_y is not None:
                cent_x = int((left_x + right_x)/2)
                cent_y = int((left_y + right_y)/2)
                cv.circle(frame, (cent_x, cent_y), 5, (0, 0, 125), -1, cv.LINE_8)

            cv.putText(frame, f"ID: {ids[0]} Distance: {np.linalg.norm(tvec):.2f}m", tuple(corners[0][0].astype(int)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    # print(f"Detected ArUco marker ID: {ids[0]}, Position: {corners[0][0]},center:[{cent_x},{cent_y}] Distance: {np.linalg.norm(tvec):.2f}m")
    cv.imshow("Aruco Markers", frame)


    # height, width, _ = frame.shape
    # hgeight = int(height / 2)
    # width = int(width / 2)
    # recent_coords = mc.get_coords()
    # target = []
    # y_plus = [-1,0,0,0,0,0]
    # y_minus = [1,0,0,0,0,0]
    # x_plus = [0,-1,0,0,0,0]
    # x_minus = [0,1,0,0,0,0]
    # if cent_y != None or cent_x != None:
    #     if height < cent_y :
    #         for i in range(len(recent_coords)):
    #             target.append(recent_coords[i] + y_plus[i])
    #     elif height > cent_y :
    #         for i in range(len(recent_coords)):
    #             target.append(recent_coords[i] + y_minus[i])
    #     else :
    #         target = recent_coords
    #     print(target)
    #     mc.sync_send_coords(target, 5, 1, 7)


    # goto_coords(mc, target)
    

    
    # cv.waitKey(1)

    
def test(frame):
    height, width, _ = frame.shape
    height = int(height / 2)
    width = int(width / 2)
    global status
    if cent_y != None and cent_x != None:
        recent_coords = mc.get_coords()
        if height + 10 < cent_y and (status == "updown_OK" and status != "아래"):
        # if height < cent_y and status != "아래" :
            # mc.send_coord(1,recent_coords[0]-1,30)
            # mc.send_coords(recent_coords[0]-1,recent_coords[1], recent_coords[2],recent_coords[3], recent_coords[4], recent_coords[5], 30)
            
            mc.jog_coord(1, 0, 2)
            status = "아래"
            print("아래")

        elif height - 10 > cent_y and (status == "updown_OK" and status != "위"):
        # elif height > cent_y and status != "위" :
            
            mc.jog_coord(1, 1, 30)
            status = "위"
            print("위")

        # elif width + 5 < cent_x and status != "좌":
        #     mc.jog_coord(2, 0, 60)
        #     status = "좌"
        #     print("좌")

        # elif width - 5 > cent_x and status != "우":
        #     mc.jog_coord(2, 1, 60)
        #     status = "우"
        #     print("우")
            # mc.send_coord(1,recent_coords[0]+1,30)
            # mc.send_coord(3,recent_coords[2],30)
            
        # elif width < cent_x :
        #     mc.send_coord(2,recent_coords[1]-1,30)
        #     print("오른")
        # elif width > cent_x :
        #     mc.send_coord(2,recent_coords[1]+1,30)
        #     print("왼")

        

        elif status != "updown_OK" and height == cent_y :
            # mc.send_coord(1 , 0, 30)
            mc.jog_stop()
            print("다시해라")
            status = "updown_OK"

        # elif status != "leftright_OK" and width -5 <= cent_x and width + 5 >= cent_x:
        #         # mc.send_coord(1 , 0, 30)
        #         mc.jog_stop()
        #         print("다시해라")
        #         status = "leftright_OK"
            



# def picking_control(center_x, center_y, angles):

#     if center_x >= 325:
#         y_offset = -5
#     elif center_x <= 315:
#         y_offset = 5
    

        


def camera_capture():
    # 카메라 초기화 및 설정
    cap = cv.VideoCapture(2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    detected_qr_codes = set()

    calib_data_path = "/home/jwy/catkin_ws/src/ive_ros/src/MultiMatrix.npz"
    calib_data = np.load(calib_data_path)
    print(calib_data.files)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 캡쳐할 수 없습니다.")
                continue

            aruco_scan(frame, calib_data)
            test(frame)
            
            key = cv.waitKey(1) 
            if key ==ord("q") or key ==27:
                cv.destroyAllWindows()
                break

              # 각 스캔 사이에 간격을 둡니다.

    except KeyboardInterrupt:
        print("Scanning stopped.")

    finally:
        cap.release()
        cv.destroyAllWindows()

def goto_angles(mc, angles):
    mc.send_angles(angles,20)
    i=0
    while True:
        # mc.is_in_position(angles,1일때 coords, 0일 때 angles)
        if mc.is_in_position(angles,0) == True:
            i+=1
            if i == 12:
                break
    print("도착: ", mc.get_angles())

def goto_coords(mc, angles):
    mc.send_coords(angles,20,1)
    i=0
    while True:
        # mc.is_in_position(angles,1일때 coords, 0일 때 angles)
        if mc.is_in_position(angles,1) == True:
            i+=1
            if i == 12:
                break
    print("도착: ", mc.get_coords())
        
def open_gripper(mc):
    mc.set_gripper_value(80, 80, 1)
    while True:
        if mc.is_gripper_moving() == 1:
            print(mc.get_gripper_value(1))
        elif mc.is_gripper_moving() == 0:
            print(mc.get_gripper_value(1))
            break
def close_gripper(mc):
    mc.set_gripper_value(15, 80, 1)
    while True:
        if mc.is_gripper_moving() == 1:
            print(mc.get_gripper_value(1))
        elif mc.is_gripper_moving() == 0:
            print(mc.get_gripper_value(1))
            break
            
    
# MyCobot 인스턴스 생성
mc = MyCobot("/dev/ttyACM0", 115200)

# 초기 설정
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
# mc.set_gripper_mode(0)
# mc.init_eletric_gripper()
time.sleep(3)

# 피킹위치로 이동
# goto_coords(mc, [142.1, 4.5, 320, -180, 0.0, -90])
# goto_coords(mc, [210, 0, 330, -180, 0.0, -90])
goto_coords(mc, [210, 0, 300, -180, 0.0, -90])

camera_capture()

# goto_coords(mc, [210, 0, 330, -180, 0.0, -90])
# close_gripper(mc)

# goto_angles(mc, [0, 0, 0, 0, 0, 0])
# open_gripper(mc)
# goto_coords(mc, [142.1, 4.5, 320, -180, 0.0, -90])



