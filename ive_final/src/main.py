#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# from sensor_msgs.msg import JointState
# from pymycobot.mycobot import MyCobot
# from pymycobot.genre import Coord
# from std_msgs.msg import Header
# from pynput import keyboard
# from pyzbar import pyzbar
# from cv2 import aruco
# import numpy as np
# import cv2 as cv
# import rospy
# import time
# import sys
# import os

# # 절대 경로를 PYTHONPATH에 추가
# sys.path.append('/home/jwy/catkin_ws/src/ive_final/src/')
# import pp

# # 전역 변수 선언
# left_x = left_y = right_x = right_y = cent_x = cent_y = None
# current_sequence = "odd"
# cent_x_history, cent_y_history = [], []
# status = "0"  # 서비스 통합 시 수정 필요

# def time_step(step):
#     start_time = time.time()
#     while time.time() - start_time < step:  # 5초 동안 대기
#         pass

# def aruco_scan(frame, calib_data):
#     global left_x, left_y, right_x, right_y, cent_x, cent_y, current_sequence, cent_x_history, cent_y_history
#     """아르코 마커를 스캔하고 각 마커의 위치와 아이디 정보를 화면에 표시합니다."""
#     cam_mat = calib_data["camMatrix"]
#     dist_coef = calib_data["distCoef"]

#     MARKER_SIZE = 0.02  # meters
#     marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
#     param_markers = aruco.DetectorParameters_create()

#     gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#     marker_corners, marker_IDs, _ = aruco.detectMarkers(
#         gray_frame, marker_dict, parameters=param_markers
#     )
#     depths = []
#     if marker_corners:
#         # 홀수와 짝수 마커를 분리
#         odd_markers = []
#         even_markers = []
#         for ids, corners in zip(marker_IDs.flatten(), marker_corners):
#             rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
#                 corners, MARKER_SIZE, cam_mat, dist_coef
#             )
#             # aruco.drawAxis(frame, cam_mat, dist_coef, rvec, tvec, 0.02)
#             depth = tvec[0][0][2]
#             depths.append((ids, depth))
#             if ids % 2 == 0:
#                 even_markers.append((ids, corners))
#             else:
#                 odd_markers.append((ids, corners))

#         # 현재 시퀀스에 따라 처리할 마커 결정
#         if current_sequence == "odd":
#             markers = odd_markers
#             current_sequence = "even"
#         else:
#             markers = even_markers
#             current_sequence = "odd"

#         for ids, corners in markers:
#             # rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
#             #     [corners], MARKER_SIZE, cam_mat, dist_coef
#             # )
#             aruco.drawDetectedMarkers(frame, [corners])
#             # aruco.drawAxis(frame, cam_mat, dist_coef, rVec, tVec, 0.02)

#             if ids % 2 == 0:
#                 left_x = corners[0][0][0]
#                 left_y = corners[0][0][1]
#             elif ids % 2 == 1:
#                 right_x = corners[0][0][0]
#                 right_y = corners[0][0][1]
#             elif ids is None:
#                 left_x = None
#                 left_y = None
#                 right_x = None
#                 right_y = None
#             if left_x is not None and left_y is not None and right_x is not None and right_y is not None:
#                 cent_x = int((left_x + right_x) / 2)
#                 cent_y = int((left_y + right_y) / 2)
#                 cent_x_history.append(cent_x)
#                 cent_y_history.append(cent_y)

#                 cv.circle(frame, (int(cent_x), int(cent_y)), 5, (0, 0, 125), -1, cv.LINE_8)

#             # cv.putText(frame, f"ID: {ids} Distance: {np.linalg.norm(tVec):.2f}m", tuple(corners[0][0].astype(int)),
#             #            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

#     cv.imshow("Aruco Markers", frame)
#     return int(depth * 1000)


# def calculate(frame):
#     height, width, _ = frame.shape
#     height, width = height // 2, width // 2
#     global left_x, left_y, right_x, right_y, cent_x, cent_y

#     if cent_y is not None and cent_x is not None:
#         w_px = 78 / (right_x - left_x)
#         print("w_px: ", w_px, "right_x: ", right_x, "left_x: ", left_x)
#         h_px = 78 / (right_y - left_y)
#         x = (cent_x - width) * w_px
#         y = (cent_y - height) * h_px
#         print(x, y)
#         return x, y
#     else:
#         return None, None

# def camera_capture(current_coords):
#     global status
#     cap = cv.VideoCapture(2)
#     cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
#     cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

#     calib_data_path = "/home/jwy/catkin_ws/src/ive_ros/src/MultiMatrix.npz"
#     calib_data = np.load(calib_data_path)
#     print(calib_data.files)

#     if status == "0":
#         while True:
#             ret, frame = cap.read()
#             if not ret:
#                 print("카메라 안뜸")

#             depth = aruco_scan(frame, calib_data)
#             x, y = calculate(frame)

#             key = cv.waitKey(1)
#             if key == ord("q") or key == 27:
#                 cap.release()
#                 cv.destroyAllWindows()
#                 return None

#             if x is not None and y is not None:
#                 cv.destroyAllWindows()
#                 bf_cali = current_coords
#                 status = "1"
#                 cap.release()
#                 cv.destroyAllWindows()
#                 return [bf_cali[0],
#                         bf_cali[1] + x - 10,
#                         bf_cali[2] + y + 40,
#                         90,
#                         0,
#                         90,
#                         30]
#             else: continue

            
#     # 밀기
#     elif status == "1":
#         print("밀기")
#         status = "2"
#         # time_step(3)
#         return [current_coords[0] + (depth - 100),
#                 current_coords[1],
#                 current_coords[2] - 5,
#                 current_coords[3],
#                 current_coords[4],
#                 current_coords[5],
#                 0]

#     # 올리기
#     elif status == "2":
#         status = "3"
#         print("들기")
#         # time_step(3)
#         return [current_coords[0],
#                 current_coords[1],
#                 current_coords[2] + 100,
#                 current_coords[3],
#                 current_coords[4],
#                 current_coords[5],
#                 30]
#     else:
#         return None

# def publish():
#     global status
#     rospy.init_node("mycobot_ive_node", anonymous=True)
#     port = rospy.get_param("~port", "/dev/ttyACM0")
#     baud = rospy.get_param("~baud", 115200)
#     print("port: {}, baud: {}\n".format(port, baud))

#     try:
#         mc = MyCobot(port, baud)
#         mc.send_angles([0, 0, 0, 0, 0, 0], 20)
#         mc.set_gripper_mode(0)
#         mc.init_eletric_gripper()
#         mc.set_end_type(1)
#         mc.set_tool_reference([0, -15, 165, 0, 0, 0])
#         print("Connected to MyCobot")
#         time.sleep(5)
#         mc.send_angles([29.44, -31.81, -104.67, -45.26, -60.29, 0.08], 20)
        
#     except Exception as e:
#         print(e)
#         print("Failed to connect to MyCobot")
#         exit(1)

#     pub = rospy.Publisher("joint_states", JointState, queue_size=10)
#     rate = rospy.Rate(30)  # 30Hz

#     joint_state_msg = JointState()
#     joint_state_msg.header = Header()
#     joint_state_msg.name = [
#         "joint2_to_joint1",
#         "joint3_to_joint2",
#         "joint4_to_joint3",
#         "joint5_to_joint4",
#         "joint6_to_joint5",
#         "joint6output_to_joint6",
#         "gripper_controller"
#     ]
#     joint_state_msg.velocity = [0]
#     joint_state_msg.effort = []

#     input_angles = [29.44, -31.81, -104.67, -45.26, -60.29, 0.08]

#     while not rospy.is_shutdown():
#         capture_result = camera_capture(mc.get_coords())
#         if capture_result is not None:
#             input_angles = capture_result
#             pp.goto_coords(mc, input_angles[:6])
#             mc.set_gripper_value(input_angles[6], 20)
#             status = "0"

#         joint_angles = mc.get_radians()
#         joint_angles.append((mc.get_gripper_value() / 117) - 0.7)

#         data_list = []
#         for index, value in enumerate(input_angles):
#             data_list.append(value)

#         # rospy.loginfo('{}'.format(data_list))
#         joint_state_msg.position = data_list

#         if joint_angles is not None:
#             joint_state_msg.header.stamp = rospy.Time.now()
#             joint_state_msg.position = joint_angles

#         pub.publish(joint_state_msg)

#         rate.sleep()


# if __name__ == "__main__":
#     try:
#         publish()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from std_msgs.msg import Header
from pynput import keyboard
from pyzbar import pyzbar
from cv2 import aruco
import numpy as np
import cv2 as cv
import rospy
import time
import sys
import os
import threading

# 절대 경로를 PYTHONPATH에 추가
sys.path.append('/home/jwy/catkin_ws/src/ive_final/src/')
import pp

# 전역 변수 선언
left_x = left_y = right_x = right_y = cent_x = cent_y = None
current_sequence = "odd"
cent_x_history, cent_y_history = [], []
status = "0"  # 서비스 통합 시 수정 필요


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
    global left_x, left_y, right_x, right_y, cent_x, cent_y

    if cent_y is not None and cent_x is not None:
        w_px = 78 / (right_x - left_x)
        h_px = 78 / (right_y - left_y)
        x = (cent_x - width) * w_px
        y = (cent_y - height) * h_px
        return x, y
    else:
        return None, None

def camera_capture():
    global status
    cap = cv.VideoCapture(2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    calib_data_path = "/home/jwy/catkin_ws/src/ive_final/src/calib_data/MultiMatrix.npz"
    calib_data = np.load(calib_data_path)

    # if not cap.isOpened():
    #     print("카메라를 열 수 없습니다.")

    try:
        while True:
            ret, frame = cap.read()
            if ret == False:
                print("프레임을 읽을 수 없습니다.")

            depth = aruco_scan(frame, calib_data)
            x, y = calculate(frame)

            if x != None and y != None and depth != None:
                cv.destroyAllWindows()
                break
            else:
                pass

            # key = cv.waitKey(1) 
            # if key == ord("q") or key == 27:
            #     cv.destroyAllWindows()
            #     break
    except KeyboardInterrupt:
        print("Scanning stopped.")

    finally:
        cap.release()
        cv.destroyAllWindows()
        return x,y,depth
 
    # if status == "1":
    #     status = "2"
    #     return [current_coords[0] + (depth - 100),
    #             current_coords[1],
    #             current_coords[2] - 5,
    #             current_coords[3],
    #             current_coords[4],
    #             current_coords[5],
    #             0]
    # elif status == "2":
    #     status = "3"
    #     return [current_coords[0],
    #             current_coords[1],
    #             current_coords[2] + 100,
    #             current_coords[3],
    #             current_coords[4],
    #             current_coords[5],
    #             30]
    # else: None
    

    
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

def publish():
    global status
    rospy.init_node("mycobot_ive_node", anonymous=True)
    port = rospy.get_param("~port", "/dev/ttyACM2")
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))

    try:
        mc = MyCobot(port, baud)
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        mc.set_gripper_mode(0)
        mc.init_eletric_gripper()
        mc.set_end_type(1)
        mc.set_tool_reference([0, -15, 165, 0, 0, 0])
        print("Connected to MyCobot")
        time.sleep(5)
        mc.send_angles([29.44, -31.81, -104.67, -45.26, -60.29, 0.08], 20)
        time.sleep(5)

    except Exception as e:
        print(e)
        print("Failed to connect to MyCobot")
        exit(1)

    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz

    joint_state_thread = threading.Thread(target=publish_joint_states, args=(mc, pub, rate))
    joint_state_thread.start()

    input_angles = [29.44, -31.81, -104.67, -45.26, -60.29, 0.08]

    while not rospy.is_shutdown():
        
        
        
        capture_result = camera_capture()
        if capture_result is not None:
            x, y, depth = capture_result
            # input_angles = capture_result
            current_coords = mc.get_coords()
            mc.send_coords([current_coords[0],             
                            current_coords[1] + x -10,        
                            current_coords[2] + y +40,        
                            90,
                            0,
                            90], 20)
            mc.set_gripper_value(30, 20)
            time.sleep(5)
            # status = "0"

        rospy.sleep(1)

if __name__ == "__main__":
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
