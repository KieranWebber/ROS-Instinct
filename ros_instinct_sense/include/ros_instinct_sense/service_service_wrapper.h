#pragma once

#include "sense_service.h"
#include <instinct_msgs/SenseValue.h>

/**
 * Wrapper for the sense service that uses ROS services to provide networked access to sense requests.
 * Service is advertised using the provided node handle at /sense_service
 */
class SenseServiceWrapper
{
  protected:
    SenseService* service_;
    // Used to deconstruct the service with the service wrapper
    ros::ServiceServer rosService_;
    ros::NodeHandle handle_;

  public:
    /**
     * @param handle Node handle for advertising services and topic subscription
     * @param service Pointer to the SenseService to query when a service request is received
     */
    SenseServiceWrapper(ros::NodeHandle& handle, SenseService* service,
                        const std::string& serviceName = "sense_service")
      : service_(service)
    {
        handle_ = handle;
        rosService_ = handle.advertiseService(serviceName, &SenseServiceWrapper::readSense, this);
    }

    /**
     * Callback registered to the advertised ROS service.
     * Parses the input from the request, reads the latest value from the SenseService and sends the output as the
     * response
     *
     * @param req ROS Service request
     * @param res ROS Service response
     * @return True if service request was completed successfully
     */
    bool readSense(instinct_msgs::SenseValueRequest& req, instinct_msgs::SenseValueResponse& res)
    {
        int senseValue = service_->readSense(req.sense_id);
        res.sense_value = senseValue;
        return true;
    }
};
