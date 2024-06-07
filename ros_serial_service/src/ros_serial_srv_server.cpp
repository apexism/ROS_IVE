#include "ros/ros.h"
#include "ros_serial_service/SrvSerial.h"

#include <serial/serial.h>

serial::Serial ser;

bool calculation(ros_serial_service::SrvSerial::Request &req, ros_serial_service::SrvSerial::Response &res) {
    if(req.motor == 0){
        ROS_INFO("request: motor stop");
        // 입력이 0일 경우 아두이노로 'a' 보내기
        ser.write("a"); // 수정: 바이트 문자열로 'a' 전송
    }else if(req.motor == 1){
        ROS_INFO("request: motor start");
        // 입력이 1일 경우 아두이노로 'b' 보내기
        ser.write("b"); // 수정: 바이트 문자열로 'b' 전송
    }

    res.result = 20;
    ROS_INFO("request.motor= %ld", (long int)req.motor);

    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ros_serial_srv_server");
    ros::NodeHandle nh;
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(timeout);
    ser.open();
    if(!ser.isOpen()){
        ROS_ERROR("Serial port could not be opened.");
        return -1;
    }

    ros::ServiceServer service = nh.advertiseService("ros_ser_srv", calculation);
    ROS_INFO("Ready to start srv server!");
    ros::spin();

    return 0;
}
