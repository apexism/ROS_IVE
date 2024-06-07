#!/usr/bin/env python3
import rospy
from ive_msg.msg import Angles

def callback(data):
    rospy.loginfo("Received angles: x={}, y={}, z={}, rx={}, ry={}, rz={}, speed={}, model={}".format(
        data.x, data.y, data.z, data.rx, data.ry, data.rz, data.speed, data.model))
    rospy.loginfo("gripper value: {}".format(
        data.gv
    ))

def listener():
    rospy.init_node("sync_plan_subscriber", anonymous=True)
    rospy.Subscriber("joint_states", Angles, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
