#include <iostream>
#include <instinct_msgs/SenseValue.h>
#include "ros_instinct_core/ros_instinct_senses.h"
#include <instinct_msgs/InstinctSenseRequest.h>
RosInstinctSenses::RosInstinctSenses(ros::NodeHandle& rosHandle, const std::string& serviceName) : rosHandle_(rosHandle)
{
    // Use a persistent service client as it is around 10x faster than reconnecting each time
    ROS_INFO("creating sense client %s", serviceName.c_str());
    client_ = rosHandle_.serviceClient<instinct_msgs::SenseValue>(serviceName, true);
    pub_ = rosHandle_.advertise<instinct_msgs::InstinctSenseRequest>("log/sense", 0);
    senseLogMessage_.sense_service = serviceName;
}

int RosInstinctSenses::readSense(const Instinct::senseID nSense)
{
    ROS_INFO_THROTTLE(0.1, "Reading Sense %d", nSense);
    senseCount_++;
    if (!client_.isValid())
    {
        // Warn about disconnection
        ROS_INFO("Missing sense service!");
        // Reconnect by creating the link
        client_ = rosHandle_.serviceClient<instinct_msgs::SenseValue>(client_.getService(), true);
    }
    instinct_msgs::SenseValueRequest req;
    instinct_msgs::SenseValueResponse res;
    req.sense_id = nSense;
    bool callRes = client_.call(req, res);
    // Any errors whilst making the call?
    if (callRes)
    {
        ROS_INFO_THROTTLE(0.1, "Sense %d Result: %d", nSense, res.sense_value);
        senseLogMessage_.sense_id = nSense;
        senseLogMessage_.sense_result = res.sense_value;
        pub_.publish(senseLogMessage_);
        return res.sense_value;
    }
    else
    {
        ROS_ERROR("Error in sense service call! Connection may be broken.");
        return -1;
    }
}

long RosInstinctSenses::getSenseCount() const
{
    return senseCount_;
}
