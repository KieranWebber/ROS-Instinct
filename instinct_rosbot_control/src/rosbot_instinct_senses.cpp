//
// Created by kieranwebber on 09/03/2020.
//

#include "../include/instinct_rosbot_control/rosbot_instinct_senses.h"
#include <instinct_rosbot_control/rosbot_senses.h>
#include <instinct_utility/movement_utility.h>

RosbotSenseService::RosbotSenseService(ros::NodeHandle& handle) : SenseService(handle)
{
    rangeBufferFL_.subscribe("/range/fl", handle);
    rangeBufferFR_.subscribe("/range/fr", handle);
    rangeBufferBL_.subscribe("/range/rl", handle);
    rangeBufferBR_.subscribe("/range/rr", handle);
    imuBuffer_.subscribe("/imu", handle);
    batteryBuffer_.subscribe("/battery", handle);
}

// Average the values from the range buffer to suppress errors from the simulator.
// Can be removed when working with the real robot.
double averageBuffer(RosMessageBuffer<sensor_msgs::Range>& buffer)
{
    double range = 0;
    int count = 0;
    if (buffer.getBufferedMessages().empty())
    {
        return 0;
    }
    count = buffer.size();
    // Take a copy of the buffer to stop it being mutated
    auto messages = buffer.getBufferedMessages();
    for (int j = 0; j < count; ++j)
    {
        // Values below this are errors from the simulation.
        // Ancedotally these appear to always be above the max so clamp them
        if (messages[j]->range < messages[j]->min_range)
        {
            range += messages[j]->max_range;
        }
        else
        {
            range += messages[j]->range;
        }
    }
    return range / count;
}

int RosbotSenseService::readSense(int senseId)
{
    ROS_DEBUG("Reading sense ID %d", senseId);
    switch (senseId)
    {
        case rosbot_instinct::senses::FRONT_LEFT_RANGE:
            return round(averageBuffer(rangeBufferFL_) * 100.f);
        case rosbot_instinct::senses::FRONT_RIGHT_RANGE:
            return round(averageBuffer(rangeBufferFR_) * 100.f);
        case rosbot_instinct::senses::BACK_LEFT_RANGE:
            return round(averageBuffer(rangeBufferBL_) * 100.f);
        case rosbot_instinct::senses::BACK_RIGHT_RANGE:
            return round(averageBuffer(rangeBufferBR_) * 100.f);
        case rosbot_instinct::senses::MIN_FRONT_RANGE: {
            int finalValue = round(std::min(averageBuffer(rangeBufferFL_), averageBuffer(rangeBufferFR_)) * 100.f);
            return finalValue;
        }
            // TODO - Cache the conversion
        case rosbot_instinct::senses::IMU_PITCH:
            return round(MovementUtility::rpyFromQuaternion(imuBuffer_.getRecentOrWait()->orientation).pitch *
                         (180.0f / M_PI));
        case rosbot_instinct::senses::IMU_ROLL:
            return round(MovementUtility::rpyFromQuaternion(imuBuffer_.getRecentOrWait()->orientation).roll *
                         (180.0f / M_PI));
        case rosbot_instinct::senses::IMU_YAW:
            return round(MovementUtility::rpyFromQuaternion(imuBuffer_.getRecentOrWait()->orientation).yaw *
                         (180.0f / M_PI));

            // Battery voltage is available on the real robot but not on the simulation so return a fixed value
        case rosbot_instinct::senses::BATTERY_VOLTAGE:
            return batteryBuffer_.size() > 0 ? batteryBuffer_.getRecentOrWait()->voltage * 100.f : 9000;
        case rosbot_instinct::senses::BATTERY_CURRENT:
            return batteryBuffer_.size() > 0 ? batteryBuffer_.getRecentOrWait()->current * 100.f : 3000;

        default:
            ROS_ERROR("Unimplemented sense %d", senseId);
            return -1;
    }
}
