#include "ros/ros.h"
#include "ros_topic/Message1.h"

void msgCallback(const ros_topic::Message1::ConstPtr&msg){
    ROS_INFO("recieve msg = %d", msg->stamp.sec);
    ROS_INFO("recieve msg = %d", msg->stamp.nsec);
    ROS_INFO("recieve msg = %d", msg->data);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ros_topic_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber ros_topic_sub = nh.subscribe("ros_topic_msg", 100, msgCallback);
    ros::spin();

    return 0;
}