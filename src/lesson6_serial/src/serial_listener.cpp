#include <ros/ros.h>
#include <std_msgs/String.h>

#include <serial/serial.h>

serial::Serial ros_serial;

void serialCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Write To Serial Port [%s]", msg->data.c_str());
    // 向串口写数据
    ros_serial.write(msg->data);
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "serial_node");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建订阅主题
    ros::Subscriber serial_sub = n.subscribe("serial_node", 1000, serialCallback);

    try
    {
        // 串口配置，并打开串口
        ros_serial.setPort("/dev/pts/22");
        ros_serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_serial.setTimeout(to);
        ros_serial.open();
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR("Unable To Open Serial Port");
        return -1;
    }

    // 设置循环的频率为10Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // 如果有数据就打印
        if (ros_serial.available())
        {
            std_msgs::String serial_data;
            serial_data.data = ros_serial.read(ros_serial.available());
            ROS_INFO("Reading From Serial Port: [%s]", serial_data.data.c_str());
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
}