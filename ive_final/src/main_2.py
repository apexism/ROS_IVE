#!/usr/bin/env python
# -*- coding:utf-8 -*-
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from std_msgs.msg import Header
from pynput import keyboard
from pyzbar import pyzbar
from cv2 import aruco
import threading
import rospy
import pp
import numpy as np
import cv2 as cv
import time
import sys
import os

# 절대 경로를 PYTHONPATH에 추가
sys.path.append('/home/jwy/catkin_ws/src/ive_final/src')

# 전역 변수 선언
left_x = left_y = right_x = right_y = cent_x = cent_y = None
status = "updown_OK"
status_2 = None
current_sequence = "odd"
cent_x_history, cent_y_history = [], []

def aruco_scan(frame, calib_data):
    global left_x, left_y, right_x, right_y, cent_x, cent_y, current_sequence, cent_x_history, cent_y_history
    """아르코 마커를 스캔하고 각 마커의 위치와 아이디 정보를 화면에 표시합니다."""
    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]

    MARKER_SIZE = 0.02 # meters
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    param_markers = aruco.DetectorParameters_create()

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    depths = []
    if marker_corners:
        # 홀수와 짝수 마커를 분리
        odd_markers = []
        even_markers = []
        for ids, corners in zip(marker_IDs.flatten(), marker_corners):
            
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, cam_mat, dist_coef
            )
            # aruco.drawAxis(frame, cam_mat, dist_coef, rvec, tvec, 0.02)
            depth = tvec[0][0][2]
            depths.append((ids, depth))
            if ids % 2 == 0:
                even_markers.append((ids, corners))
            else:
                odd_markers.append((ids, corners))

        # 현재 시퀀스에 따라 처리할 마커 결정
        if current_sequence == "odd":
            markers = odd_markers
            current_sequence = "even"
        else:
            markers = even_markers
            current_sequence = "odd"

        for ids, corners in markers:
            # rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            #     [corners], MARKER_SIZE, cam_mat, dist_coef
            # )
            aruco.drawDetectedMarkers(frame, [corners])
            # aruco.drawAxis(frame, cam_mat, dist_coef, rVec, tVec, 0.02)

            if ids % 2 == 0:
                left_x = corners[0][0][0]
                left_y = corners[0][0][1]
            elif ids % 2 == 1:
                right_x = corners[0][0][0]
                right_y = corners[0][0][1]
            elif ids is None:
                left_x = None
                left_y = None
                right_x = None
                right_y = None
            if left_x is not None and left_y is not None and right_x is not None and right_y is not None:
                cent_x = int((left_x + right_x) / 2)
                cent_y = int((left_y + right_y) / 2)
                cent_x_history.append(cent_x)
                cent_y_history.append(cent_y)

                cv.circle(frame, (int(cent_x), int(cent_y)), 5, (0, 0, 125), -1, cv.LINE_8)

            # cv.putText(frame, f"ID: {ids} Distance: {np.linalg.norm(tVec):.2f}m", tuple(corners[0][0].astype(int)),
                    #    cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    cv.imshow("Aruco Markers", frame)
    return int(depth*1000)


def calculate(frame):
    height, width, _ = frame.shape
    height, width = height // 2, width // 2
    global left_x, left_y, right_x, right_y, status, status_2, cent_x, cent_y


    if cent_y is not None and cent_x is not None:
        w_px = 78 / (right_x - left_x)
        print("w_px: ", w_px, "right_x: ",right_x, "left_x: ", left_x)
        h_px = 78 / (right_y - left_y)
        x = (cent_x - width) * w_px
        y = (cent_y - height) * h_px
        print(x,y)
        return x, y
    else: return None, None
   
def camera_capture():
    # 카메라 초기화 및 설정
    cap = cv.VideoCapture(2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    # detected_qr_codes = set()
    calib_data_path = "/home/jwy/catkin_ws/src/ive_ros/src/MultiMatrix.npz"
    calib_data = np.load(calib_data_path)
    print(calib_data.files)

    try:
        while True:
            ret, frame = cap.read()
            if ret == False:
                print("카메라 안뜸")

            depth = aruco_scan(frame, calib_data)
            x, y = calculate(frame)
            
            if x != None and y != None:

                cv.destroyAllWindows()
                mc.set_end_type(1)
                mc.set_tool_reference([0,-15,165,0,0,0])
                print("test")
                bf_cali = mc.get_coords()
                mc.send_coords([bf_cali[0] + x -10 ,             
                                bf_cali[1] ,        
                                bf_cali[2] + y + 50,        
                                90,
                                0,
                                0,                        
                                ], 20, 1)
                time.sleep(5)
                break
            else: pass

            key = cv.waitKey(1) 
            if key == ord("q") or key == 27:
                cv.destroyAllWindows()
                break
            elif key == ord("w") or key ==87 or key == 119:
                mc.jog_stop()

    except KeyboardInterrupt:
        print("Scanning stopped.")

    finally:
        cap.release()
        cv.destroyAllWindows()
    return depth

def publish_joint_states(mc, pub, rate):
    joint_state_msg = JointState()
    joint_state_msg.header = Header()
    joint_state_msg.name = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
        "gripper_controller"
    ]
    joint_state_msg.velocity = [0]
    joint_state_msg.effort = []

    while not rospy.is_shutdown():
        joint_angles = mc.get_radians()
        joint_angles.append((mc.get_gripper_value() / 117) - 0.7)
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = joint_angles
        pub.publish(joint_state_msg)
        rate.sleep()

# MyCobot 인스턴스 생성
rospy.init_node("mycobot_ive_node", anonymous=True)
port = rospy.get_param("~port", "/dev/ttyACM0")
baud = rospy.get_param("~baud", 115200)
print("port: {}, baud: {}\n".format(port, baud))


mc = MyCobot(port, baud)

pub = rospy.Publisher("joint_states", JointState, queue_size=10)
rate = rospy.Rate(30)  # 30Hz

joint_state_thread = threading.Thread(target=publish_joint_states, args=(mc, pub, rate))
joint_state_thread.start()

mc.send_angles([0, 0, 0, 0, 0, 0], 40)
# mc.send_coords([180, -200, 90, 90, 0, 0], 30)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
mc.set_end_type(1)
mc.set_tool_reference([0, -15, 165, 0, 0, 0])
print("Connected to MyCobot")
time.sleep(5)
# mc.send_angles([29.44, -31.81, -104.67, -45.26, -60.29, 0.08], 20)
# mc.send_angles([-36.21, -57.83, -68.64, -60.38, -29.61, 2.63],20)
# mc.send_coords([80, -400, 55, 90, 0, 0], 30)
# mc.send_angles([-0.43, -50.97, -62.75, -78.48, 0.7, 14.06], 20) # 각도
# mc.send_coords([190, -320, 110, 90, 0, 0],20) #촬영 좌표
# ㅁ각도:  [0.26, -38.05, -86.04, -60.2, -1.4, 2.02]
# mc.send_coords([180, -250, 90, 90, 0, 0], 30)
# mc.send_angles([36.29, -36.29, -104.5, -42.01, 34.1, -0.17],30)
# mc.send_coords([180, -210, 70, 90, 0, 0], 30)
# 각도:  [3.16, -48.86, -74.09, -82.7, 0.17, 21.44]
# mc.send_coords([170, -310, 80, 90, 0, 0], 30)
# 각도:  [15.2, -58.97, -66.09, -41.74, 13.53, -17.66]
# mc.send_coords([257.5, -252.4, 59.7, 90, 0, 0], 30)
# 각도:  [74.7, 59.23, 78.48, 42.53, -16.25, 0.17]
# mc.send_coords([260, -210, 50, 90, 0, 90], 30)
# 각도:  [24.25, -56.77, -74.44, -48.33, 23.64, -0.87]
mc.send_coords([225, -225, 60, 90, 0, 0], 30, 1)
time.sleep(5)



print(mc.get_coords())
depth = camera_capture()
print(depth)
pp.mid_gripper(mc)


#밀기
current_coords = mc.get_coords()
print("Current Coordinates: ", current_coords)


mc.send_coords([current_coords[0] ,
                current_coords[1] - (depth -100),
                current_coords[2] - 5 ,
                current_coords[3],
                current_coords[4],
                current_coords[5],], 20, 1)
time.sleep(5)
pp.close_gripper(mc)

# 올리기
# 각도:  [3.33, -41.57, -58.44, -91.58, -1.4, 12.74]
# mc.send_coords([212.5, -306.2, 149.8, 90.28, -1.14, 4.71], 30, 1)
current_coords = mc.get_coords()
mc.send_coords([current_coords[0],
                current_coords[1] - 70,
                current_coords[2] + 100,
                current_coords[3],
                current_coords[4],
                current_coords[5],], 20)
time.sleep(5)

# mc.send_angles([29.44, -31.81, -104.67, -45.26, -60.29, 0.08], 20)
# mc.send_coords([399.7, -5.6, 130, 90, 0, 90], 20, 1)
# 컨베이어 벨트 위로 이동
# 각도:  [25.13, -45.61, -21.53, -117.24, -66.88, -2.19]
mc.send_coords([447.2, 12.5, 194, 90, 0, 90], 20, 1)
time.sleep(5)
# 그리퍼 여는 위치
# 각도:  [23.29, -79.1, -16.08, -87.09, -68.29, -1.31]
mc.send_coords([492.6, 23.4, 90, 90, 0, 90], 20, 1)
time.sleep(5)
pp.mid_gripper(mc)