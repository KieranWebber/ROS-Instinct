//
// Created by kieranwebber on 09/03/2020.
//

#pragma once

#include <ros_instinct_sense/sense_service.h>
#include <instinct_utility/ros_message_buffer.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>

class RosbotSenseService : public SenseService
{
#define TOPIC_BUFFER_SIZE 2

  public:
    RosbotSenseService(ros::NodeHandle& handle);
    virtual int readSense(int senseId) override;

  private:
    // Range sensors
    RosMessageBuffer<sensor_msgs::Range> rangeBufferFL_{ TOPIC_BUFFER_SIZE };
    RosMessageBuffer<sensor_msgs::Range> rangeBufferFR_{ TOPIC_BUFFER_SIZE };
    RosMessageBuffer<sensor_msgs::Range> rangeBufferBL_{ TOPIC_BUFFER_SIZE };
    RosMessageBuffer<sensor_msgs::Range> rangeBufferBR_{ TOPIC_BUFFER_SIZE };

    // Pose Messages
    RosMessageBuffer<sensor_msgs::Imu> imuBuffer_{ TOPIC_BUFFER_SIZE };

    // Battery Messages
    RosMessageBuffer<sensor_msgs::BatteryState> batteryBuffer_{ TOPIC_BUFFER_SIZE };
};
