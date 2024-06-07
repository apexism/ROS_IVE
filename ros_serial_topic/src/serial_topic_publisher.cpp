#include "ros/ros.h"
#include "ros_serial_topic/MsgSerial.h" // MsgTutorial Message File Header.
#include <serial/serial.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "serial_topic_publisher"); // Initializes Node Name
    ros::NodeHandle nh;
    ros::Publisher ros_serial_pub = nh.advertise <ros_serial_topic::MsgSerial>("ros_serial_msg", 100);

    ros::Rate loop_rate(10);

    ros_serial_topic::MsgSerial msg;

    serial::Serial ser;
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(timeout);
    ser.open();

    if (!ser.isOpen()){
        ROS_ERROR("Serial port could not be opened.");
        return -1;
    }

    while (ros::ok()){
        if (ser.available()>0){
            std::string data_str = ser.readline();
            int data = std::stof(data_str);
            msg.temp = (float)data;
            ROS_INFO("send msg = %f", msg.temp);
            ros_serial_pub.publish(msg);
        }
        loop_rate.sleep();
    }
    return 0;

} 
