#include <ros/ros.h>

#include "keyboard_reader/Key.h"

void keyboardCallback(const keyboard_reader::Key::ConstPtr &key)
{
    ROS_INFO("Key_Code = %d, Key_Name = %d, Key_Pressed = %d", key->key_code, key->key_name, key->key_pressed);
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "ros_keyboard");

    // 初始化节点句柄
    ros::NodeHandle n;

    // 创建订阅节点
    ros::Subscriber sub = n.subscribe("keyboard", 1000, keyboardCallback);

    // 处理所有回调函数
    ros::spin();

    return 0;
}