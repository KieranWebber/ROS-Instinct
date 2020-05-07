#include <ros_instinct_actions/instinct_action_service.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "instinct_actions");
    ros::NodeHandle n;
    ROS_INFO("Starting action service.");
    ros::spin();
    return 0;
}