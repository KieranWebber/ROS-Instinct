#include <ros_instinct_actions/instinct_action_status.h>
#include "../include/instinct_benchmark/simple_action.h"
unsigned char SimpleAction::executeAction(const int actionId, const int actionValue)
{
    // ROS_INFO("Completing Action!");
    // Toggle this depending on the benchmark
    // ros::Duration(actionValue / 1000.0).sleep();
    return ros_instinct::action::INSTINCT_SUCCESS;
}
SimpleAction::SimpleAction(ros::NodeHandle& nh) : InstinctActionService(nh)
{
}
