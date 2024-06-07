#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pymycobot.mycobot import MyCobot
from std_msgs.msg import String
import time

class PickPlaceNode:
    def __init__(self):
        self.mc = MyCobot('/dev/ttyACM0', 115200)
        rospy.init_node('pick_place_node', anonymous=True)
        self.command_sub = rospy.Subscriber('pick_command', String, self.command_callback)

    def command_callback(self, msg):
        rospy.loginfo("Received command to pick: %s", msg.data)
        self.pick_object(msg.data)

    def pick_object(self, color):
        # 여기에 객체를 집고 놓는 코드 추가
        rospy.loginfo("Picking up %s object", color)
        time.sleep(2)
        rospy.loginfo("%s object picked and placed", color)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PickPlaceNode()
    node.run()

