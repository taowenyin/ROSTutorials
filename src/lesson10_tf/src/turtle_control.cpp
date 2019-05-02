#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

void poseCallback(const turtlesim::PoseConstPtr &msg)
{
    ROS_INFO("Turtle2 position: (%f, %f)", msg->x, msg->y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_control");
    ros::NodeHandle n;
    ros::Publisher turtle_vel;

    // 通过服务调用，产生第二只乌龟
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = n.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    srv.request.x = 5;
    srv.request.y = 5;
    srv.request.name = "turtle2";
    add_turtle.call(srv);

    ROS_INFO("The turtle2 has been spawn.");

    // 订阅乌龟的位置信息
    ros::Subscriber sub = n.subscribe("turtle2/pose", 10, &poseCallback);

    // 定义乌龟的速度控制发布器
    turtle_vel = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(10.0);

    while (n.ok())
    {
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 1;
        vel_msg.linear.x = 1;
        turtle_vel.publish(vel_msg);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}