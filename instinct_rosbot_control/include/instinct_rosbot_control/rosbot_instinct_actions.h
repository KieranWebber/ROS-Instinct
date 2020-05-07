//
// Created by kieranwebber on 01/03/2020.
//

#pragma once  // SRC_ROSBOT_INSTINCT_ACTIONS_H
#include <ros/ros.h>
#include <ros_instinct_actions/instinct_action_service.h>
#include <instinct_utility/ros_message_buffer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

/**
 * Range from the target angle that turning is allowed to stop
 */
#define TURN_THRESHOLD 3

/**
 * Action service for the ROSBot robot.
 * Handles locomotion commands from the Instinct planner
 */
class RosbotActionService : public InstinctActionService
{
  public:
    RosbotActionService(ros::NodeHandle& handle, float turnSpeed = M_PI_2);
    virtual unsigned char executeAction(const int actionId, const int actionValue) override;
    void publishCommands();
    /**
     * Compute and set the angles required to make the given turn amount
     * @param angle Amount to turn
     */
    void computeTurn(int angle);

  protected:
    // Publisher + timer used for submitting Twist commands on interval for locomotion
    ros::Publisher pub_;
    ros::Timer pubTimer_;
    int speed_ = 0;
    /**
     * Speed in radians to use when turning the robot
     */
    float turnSpeed_;
    /**
     * Current turning speed.
     */
    float turnRate_ = 0;
    float desiredYaw = 0;

    // IMU messages required to perform turning
    // Only need to cache the last message
    RosMessageBuffer<sensor_msgs::Imu> imuBuffer_{ 1 };
    RosMessageBuffer<sensor_msgs::Range> rangeBufferFL_{ 1 };
    RosMessageBuffer<sensor_msgs::Range> rangeBufferFR_{ 1 };

    bool turnAngle(int angle);
};
