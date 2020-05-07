#pragma once

#include <ros_instinct_actions/instinct_action_service.h>
#include <ros_instinct_sense/sense_service.h>
#include <ros_instinct_sense/service_service_wrapper.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
/**
 * Integer key value store utilising the ROS-Instinct framework.
 *
 * Actions are advertised on store_action_service
 * Senses are advertised on store_sense_service
 */
class InstinctStoreService : public InstinctActionService, public SenseService
{
  public:
    /**
     * @param nh Node handle to use during registration of action server and services
     * @param defaultValue Default value to return on sense if none is stored
     */
    InstinctStoreService(ros::NodeHandle& nh, int defaultValue = 0);
    virtual int readSense(int senseId) override;
    /**
     * Remove all stored values within the store
     */
    void resetStore();
    /**
     * Return the value with the given ID or the default if it doesnt exist
     * @param senseId Store id to query
     * @return Stored value or default if it doesnt exist
     */
    int getValueOrDefault(int senseId);

  protected:
    boost::shared_mutex mutex_;
    virtual unsigned char executeAction(const int actionId, const int actionValue) override;
    std::map<int, int> valueMap_;
    int defaultValue_ = 0;
    SenseServiceWrapper wrapper_;
};
