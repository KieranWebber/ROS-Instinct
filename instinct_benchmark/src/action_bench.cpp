#include <ros_instinct_actions/instinct_action_service.h>
#include <ros/ros.h>
#include "../include/instinct_benchmark/simple_action.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "instinct_actions");
    ros::NodeHandle n;
    SimpleAction actionService{ n };
    ros::spin();
}