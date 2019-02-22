#include <ros/ros.h>
#include <std_msgs/String.h>

// 订阅的回调函数
void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("ROS Receive [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "listener");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建订阅节点
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // 处理所有回调函数
    ros::spin();

    return 0;
}