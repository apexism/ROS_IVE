#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pymycobot.mycobot import MyCobot
from std_msgs.msg import String
import time

class MainNode:
    def __init__(self):
        self.mc = MyCobot('/dev/ttyACM0', 115200)
        rospy.init_node('main_node', anonymous=True)
        self.color_sub = rospy.Subscriber('color_detect', String, self.color_callback)
        self.pick_pub = rospy.Publisher('pick_command', String, queue_size=10)
        self.init_robot()

    def init_robot(self):
        self.mc.send_angles([0,0,0,0,0,0], 60)
        time.sleep(5)
        self.mc.set_gripper_mode(0)
        self.mc.init_eletric_gripper()
        time.sleep(1)
        self.mc.set_end_type(1)
        self.mc.set_tool_reference([0,0,0,0,0,0])
        time.sleep(1)
        rospy.loginfo("Robot Initialized")

    def color_callback(self, msg):
        rospy.loginfo("Detected color: %s", msg.data)
        self.pick_pub.publish(msg.data)
        time.sleep(5)  # Simulate time to pick

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = MainNode()
    node.run()
