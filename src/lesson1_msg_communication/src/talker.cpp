#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "talker");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个发布者，并命名为chatter，同时设置队列缓存为1000
    ros::Publisher chatter = n.advertise<std_msgs::String>("chatter", 1000);

    // 设置循环的频率为10Hz
    ros::Rate loop_rate(10);

    // 发送计数器
    int count = 0;

    // 判断ros节点准备完毕
    while (ros::ok())
    {
        // 创建要发送对象的消息
        std_msgs::String msg;

        // 创建要发送的消息
        std::stringstream ss;
        ss << "Hello ROS " << count++;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        // 发布消息
        chatter.publish(msg);

        // 处理所有回调函数
        ros::spinOnce();

        // 节点休眠
        loop_rate.sleep();
    }

    return 0;
}