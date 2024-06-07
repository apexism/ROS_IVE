#include "ros/ros.h"
#include "ros_serial_service/SrvSerial.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_serial_srv_client");

    if(argc != 2)
    {
        ROS_INFO("cmd : rosrun ros_serial_service ros_serial_srv_client arg0");
        ROS_INFO("arg0: 0: motor stop, 1: motor start");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ros_serial_service::SrvSerial>("ros_ser_srv");
    ros_serial_service::SrvSerial srv;
    srv.request.motor = atoll(argv[1]);
    if(client.call(srv))
    {
        ROS_INFO("send srv, srv.request.motor: %ld", (long int)srv.request.motor);
        ROS_INFO("receive srv, srv.response.result: %ld", (long int)srv.response.result);
    }
    else
    {
        ROS_ERROR("Failed to call service ros_ser_srv");
        return 1;
    }
    return 0;
}
