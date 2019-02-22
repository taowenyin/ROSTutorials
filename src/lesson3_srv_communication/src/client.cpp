#include <ros/ros.h>

#include "lesson3_srv_communication/AddTwoInts.h"

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "add_two_ints_client");

    // 判断是否传入了2个参数
    if (argc != 3)
    {
        ROS_INFO("Usage: add_two_ints_client X Y");
        return 1;
    }

    // 创建节点句柄
    ros::NodeHandle n;

    // 添加客户端，并设置服务消息类型，以及服务名称
    ros::ServiceClient client = n.serviceClient<lesson3_srv_communication::AddTwoInts>("add_two_ints");

    // 创建服务对象内容
    lesson3_srv_communication::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    // 发布service请求，等待应答
    if (client.call(srv))
    {
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}