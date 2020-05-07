#include <boost/timer/timer.hpp>
#include <ros/ros.h>
#include <ros_instinct_sense/service_service_wrapper.h>
#include "../include/instinct_benchmark/simple_sense.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "instinct_sense");
    ros::NodeHandle n;
    SimpleSense testSense{ n };
    SenseServiceWrapper wrapper{ n, &testSense };
    ros::spin();
}
