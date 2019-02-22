#include <ros/ros.h>
#include "lesson3_srv_communication/AddTwoInts.h"

bool add(lesson3_srv_communication::AddTwoInts::Request &req, lesson3_srv_communication::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b;

    ROS_INFO("Request: x = %ld, y = %ld", (long int)req.a, (long int)req.b);
    ROS_INFO("Sending back response: [%ld]", (long int)res.sum);

    return true;
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "add_two_ints_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Service，并设置其服务的回调函数
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);

    ROS_INFO("Ready to add two ints");

    // 处理所有回调函数
    ros::spin();

    return 0;
}