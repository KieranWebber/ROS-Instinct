#include "../include/instinct_rosbot_control/rosbot_instinct_actions.h"
#include "../include/instinct_rosbot_control/rosbot_actions.h"

#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <instinct_utility/movement_utility.h>
#include <ros_instinct_actions/instinct_action_status.h>

RosbotActionService::RosbotActionService(ros::NodeHandle& handle, float turnSpeed) : InstinctActionService(handle)
{
    turnSpeed_ = turnSpeed;
    // Subscribe to any topics required to perform actions
    imuBuffer_.subscribe("imu", handle);
    rangeBufferFR_.subscribe("/range/fr", handle);
    rangeBufferFL_.subscribe("/range/fl", handle);
    // Advertise a publisher for the twist locomotion comands
    pub_ = handle.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    // Publish the locomotion message on interval
    // ROSBot requires this to keep moving
    pubTimer_ = handle.createTimer(ros::Duration(0.1), std::bind(&RosbotActionService::publishCommands, this));
}

unsigned char RosbotActionService::executeAction(const int actionId, const int actionValue)
{
    ROS_INFO("Executing action!");
    switch (actionId)
    {
        case rosbot_instinct::actions::SET_SPEED:
            speed_ = actionValue;
            break;
        case rosbot_instinct::actions::SET_TURN_RATE:
            // Turn rate is set in radians
            turnRate_ = actionValue * (M_PI / 180.0);
            break;
        case rosbot_instinct::actions::TURN:
            return turnAngle(actionValue) ? ros_instinct::action::INSTINCT_SUCCESS :
                                            ros_instinct::action::INSTINCT_ERROR;
        case rosbot_instinct::actions::OPEN_TURN: {
            // Flip the angle based on which sensor is reporting closest. Enables turning away from walls in the
            // quickest time
            int angle = (rangeBufferFR_.getMostRecent()->range < rangeBufferFL_.getMostRecent()->range) ? actionValue :
                                                                                                          -actionValue;
            return (turnAngle(angle) ? ros_instinct::action::INSTINCT_SUCCESS : ros_instinct::action::INSTINCT_ERROR);
        }
        default:
            return ros_instinct::action::INSTINCT_ERROR;
    }
    return ros_instinct::action::INSTINCT_SUCCESS;
}

void RosbotActionService::publishCommands()
{
    ROS_INFO("Publishing speed %d!", speed_);
    geometry_msgs::Twist velMsg;
    // Scale the speed into something more suited to the integer Instinct value
    velMsg.linear.x = speed_ / 20.f;
    velMsg.angular.z = turnRate_;
    pub_.publish(velMsg);
}

void RosbotActionService::computeTurn(int angle)
{
    auto reading = imuBuffer_.getRecentOrWait();
    auto orientation = reading->orientation;
    // Get the current yaw
    double yaw = MovementUtility::rpyFromQuaternion(orientation).yaw * (180 / M_PI);
    // Compute the desired yaw
    int newYaw = (int)round(yaw + angle) % 360;
    // Wrap the value between 0-360
    desiredYaw = newYaw < 0 ? 360 + newYaw : newYaw;
    // Turn in the direction of the original angle
    turnRate_ = (angle < 0 ? -1.0 : 1.0) * turnSpeed_;
}
bool RosbotActionService::turnAngle(int angle)
{
    ros::Rate r(60);
    speed_ = 0;
    computeTurn(angle);
    // Cancel the action if the node is shutdown or the action is being preempted
    while (!actionServer_.isPreemptRequested() && ros::ok())
    {
        auto imuReading = imuBuffer_.getMostRecent();
        int yaw = round(MovementUtility::rpyFromQuaternion(imuReading->orientation).yaw * (180 / M_PI));
        publishFeedback(yaw);
        // Dont spam the logs with messages
        ROS_INFO_THROTTLE(0.5, "Turn Yaw: %d [%d]", yaw, (int)desiredYaw);
        if (abs(desiredYaw - yaw) < TURN_THRESHOLD)
        {
            desiredYaw = 0;
            turnRate_ = 0;
            return true;
        }
        r.sleep();
    }
    turnRate_ = 0;
    return false;
}
