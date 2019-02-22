#include <ros/ros.h>
#include <sstream>
#include <string>

#include "lesson2_msg_communication/Person.h"

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "talker");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个发布者，并命名为chatter，同时设置队列缓存为1000
    ros::Publisher chatter = n.advertise<lesson2_msg_communication::Person>("Person", 1000);

    // 设置循环的频率为10Hz
    ros::Rate loop_rate(10);

    // 发送计数器
    int count = 0;

    // 判断ros节点准备完毕
    while (ros::ok())
    {
        // 创建要发送对象的消息
        lesson2_msg_communication::Person person_msg;

        std::stringstream ss;
        ss << "1111";

        // 创建要发送的消息
        person_msg.name = ss.str();
        person_msg.age = 20;
        person_msg.sex = 1;

        ROS_INFO("ROS talker name = %s, age = %d, sex = %d", person_msg.name, person_msg.age, person_msg.sex);

        // 发布消息
        chatter.publish(person_msg);

        // 处理所有回调函数
        ros::spinOnce();

        // 节点休眠
        loop_rate.sleep();
    }

    return 0;
}