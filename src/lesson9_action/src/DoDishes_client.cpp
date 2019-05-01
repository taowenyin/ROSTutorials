#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "lesson9_action/DoDishesAction.h"

typedef actionlib::SimpleActionClient<lesson9_action::DoDishesAction> Client;

// 当Action完成后会调用该回调函数一次
void doneCB(const actionlib::SimpleClientGoalState &state, const lesson9_action::DoDishesResultConstPtr &result)
{
    ROS_INFO("Yay! The dishes are noe clean");
    ros::shutdown();
}

// 当Action激活后会调用该回调函数一次
void activeCB()
{
    ROS_INFO("Goal just went active");
}

// 收到feedback后调用该回调函数
void feedbackCB(const lesson9_action::DoDishesFeedbackConstPtr &feedback)
{
    ROS_INFO(" percent_complete: %f", feedback->percent_complete);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Do_Dishes_Client");

    // 定义一个客户端
    Client client("do_dishes", true);

    // 等待服务端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // 创建一个action的goal
    lesson9_action::DoDishesGoal goal;
    goal.dishwasher_id = 1;

    // 发送action的goal给服务器端，并设置回调函数
    client.sendGoal(goal, &doneCB, &activeCB, &feedbackCB);

    ros::spin();

    return 0;
}