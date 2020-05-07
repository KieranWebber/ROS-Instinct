#include <ros_instinct_actions/instinct_action_status.h>
#include "../include/instinct_store/instinct_store_service.h"

int InstinctStoreService::getValueOrDefault(int senseId)
{
    if (valueMap_.find(senseId) != valueMap_.end())
    {
        return valueMap_.at(senseId);
    }
    else
    {
        return defaultValue_;
    }
}

int InstinctStoreService::readSense(int senseId)
{
    ROS_INFO("Store read request ID: %d", senseId);
    boost::shared_lock<boost::shared_mutex> guard(mutex_);
    return getValueOrDefault(senseId);
}

unsigned char InstinctStoreService::executeAction(const int actionId, const int actionValue)
{
    // Lock the mutex for writing
    boost::unique_lock<boost::shared_mutex> guard(mutex_);
    switch (actionId)
    {
        // Decrement
        case 0: {
            int value = getValueOrDefault(actionValue) - 1;
            valueMap_[actionValue] = value;
            ROS_INFO("Decrement store value ID: %d. New Value: %d", actionValue, value);
            return ros_instinct::action::INSTINCT_SUCCESS;
        }
        // Increment
        case 1: {
            int value = getValueOrDefault(actionValue) + 1;
            ROS_INFO("Increment store value ID: %d. New Value: %d", actionValue, value);
            valueMap_[actionValue] = value;
            return ros_instinct::action::INSTINCT_SUCCESS;
        }
        // Set value directly
        default:
            ROS_INFO("Store value set ID: %d, Value: %d", actionId, actionValue);
            valueMap_[actionId] = actionValue;
            return ros_instinct::action::INSTINCT_SUCCESS;
    }
}

InstinctStoreService::InstinctStoreService(ros::NodeHandle& nh, int defaultValue)
  : defaultValue_(defaultValue)
  , InstinctActionService(nh, "store_action_service")
  , SenseService(nh)
  , wrapper_(nh, this, "store_sense_service")
{
}
void InstinctStoreService::resetStore()
{
    boost::lock_guard<boost::shared_mutex> guard(mutex_);
    valueMap_.clear();
}
