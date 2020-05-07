#include <ros/ros.h>
#include <instinct_rosbot_control/rosbot_instinct_actions.h>
#include <ros_instinct_actions/instinct_action_service.h>
#include <instinct_rosbot_control/rosbot_instinct_senses.h>
#include <ros_instinct_sense/service_service_wrapper.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "instinct_actions");
    ros::NodeHandle n;
    ros::NodeHandle pNode("~");
    ROS_INFO("Starting action service.");
    float turnSpeed;
    pNode.param<float>("turn_speed", turnSpeed, M_PI_2);
    RosbotActionService actionService(n, turnSpeed);

    ROS_INFO("Starting sense service.");
    RosbotSenseService senseService(n);
    SenseServiceWrapper senseServiceWrapper(n, &senseService);

    ros::spin();
    return 0;
}