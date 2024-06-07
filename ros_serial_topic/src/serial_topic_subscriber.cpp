#include "ros/ros.h"
#include "ros_serial_topic/MsgSerial.h"

void msgCallback(const ros_serial_topic::MsgSerial::ConstPtr&msg)
{
    ROS_INFO("receive msg = %f", msg->temp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_topic_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber ros_serial_sub = nh.subscribe("ros_serial_msg", 100, msgCallback);

    ros::spin();

    return 0;
}