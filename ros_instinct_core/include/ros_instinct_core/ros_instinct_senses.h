#pragma once
#include <instinct/Instinct.h>
#include <ros/node_handle.h>
#include <instinct_msgs/InstinctSenseRequest.h>

/**
 * ROS enabled Instinct sense interface.
 * Uses ROS services to request sense values on demand over the network (blocking).
 */
class RosInstinctSenses : public Instinct::Senses
{
  protected:
    ros::NodeHandle rosHandle_;
    ros::ServiceClient client_;
    /**
     * Publishes sense result for logging to /log/sense using the InstinctSenseRequest message type
     */
    ros::Publisher pub_;
    /**
     * Message to publish when sense result is received.
     * Local copy can be reusued for efficency.
     */
    instinct_msgs::InstinctSenseRequest senseLogMessage_;
    /**
     * Total number of sense requests made since connection
     */
    long senseCount_ = 0;

  public:
    /**
     * @return Total number of sense requests made since connection
     */
    long getSenseCount() const;

  public:
    RosInstinctSenses(ros::NodeHandle& rosHandle, const std::string& serviceName = "sense_service");
    virtual int readSense(const Instinct::senseID nSense) override;
};
