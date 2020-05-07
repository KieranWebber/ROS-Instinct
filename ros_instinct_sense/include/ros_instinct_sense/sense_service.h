#pragma once  // SRC_SENSE_SERVICE_H

#include <ros/node_handle.h>
/**
 * Base class providing sense reading for the ROS-Instinct Planner.
 * Extend readSense() to provide sense results for all input IDs.
 *
 * Error handling must be done internally before returning a default value.
 */
class SenseService
{
  public:
    /**
     * @param handle Node handle use for ROS subscriptions
     */
    SenseService(const ros::NodeHandle& handle) : nodeHandle_(handle)
    {
    }
    /**
     * Read the current / most recent sense value for the given sense id.
     * Method is blocking for the planner and must return quickly.
     * A buffer/cache should be used to store complex or infrequent sense values.
     *
     *
     * @param senseId Sense ID to query
     * @return  Current value of the sense
     */
    virtual int readSense(int senseId) = 0;

  protected:
    /**
     * Local node handle copy
     */
    ros::NodeHandle nodeHandle_;
};
