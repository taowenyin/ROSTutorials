#include <ros/ros.h>

#include "lesson2_msg_communication/Person.h"

// 订阅的回调函数
void chatterCallback(const lesson2_msg_communication::Person::ConstPtr &msg)
{
    ROS_INFO("ROS Receive Person name = [%s], age = [%d], sex = [%d]", msg->name, msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "listener");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建订阅节点
    ros::Subscriber sub = n.subscribe("Person", 1000, chatterCallback);

    // 处理所有回调函数
    ros::spin();

    return 0;
}