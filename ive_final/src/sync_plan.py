#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import os

# 절대 경로를 PYTHONPATH에 추가
sys.path.append('/home/jwy/catkin_ws/src/ive_ros/src')

import time
import rospy
import picknplace as pp
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.mycobot import MyCobot
from pynput import keyboard

input_key = None

def on_press(key):
    global input_key
    try:
        if key.char == '1':
            input_key = '1'
        elif key.char == '2':
            input_key = '2'
    except AttributeError:
        pass

def on_release(key):
    global input_key
    if key == keyboard.Key.esc:
        return False

def key_input(key, mc):
    if key == '1':
        print("1이 입력되었습니다.")
        return [15, 20, 10, 10, 10, 10, 70]
    elif key == '2':
        print("2가 입력되었습니다.")
        return [0, 0, 0, 0, 0, 0, 30]
    # ESC 키가 입력되면 종료
    # else : None

def talker():
    global input_key
    rospy.init_node("mycobot_control_node", anonymous=True)
    port = rospy.get_param("~port", "/dev/ttyACM1")
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))

    try:
        mc = MyCobot(port, baud)
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        mc.set_gripper_mode(0)
        mc.init_eletric_gripper()
        print("Connected to MyCobot")
    except Exception as e:
        print(e)
        print("Failed to connect to MyCobot")
        exit(1)

    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz

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

    input_angles = [0,0,0,0,0,0,0]

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    while not rospy.is_shutdown():
        if input_key == '1' or input_key == '2':  # 키가 눌렸는지 확인
            input_angles = key_input(input_key, mc)
            mc.send_angles(input_angles[:6], 20)
            mc.set_gripper_value(input_angles[6], 20)
            input_key = None
            
        joint_angles = mc.get_radians()
        joint_angles.append((mc.get_gripper_value()/117)-0.7)

        data_list = []
        for index, value in enumerate(input_angles):
            data_list.append(value)

        # rospy.loginfo('{}'.format(data_list))
        joint_state_msg.position = data_list

        if joint_angles is not None:
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.position = joint_angles

        pub.publish(joint_state_msg)

        rate.sleep()

    listener.stop()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
