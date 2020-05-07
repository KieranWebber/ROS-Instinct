//
// Created by kieranwebber on 2/5/20.
//

#pragma once  // SRC_ROS_INSTINCT_ACTIONS_H

#include <instinct/Instinct.h>
#include <actionlib/client/simple_action_client.h>
#include <instinct_msgs/InstinctAction.h>
#include <instinct_msgs/InstinctActionRequest.h>

/**
 * ROS connected Instinct action interface
 * Uses actionlib to perform asynchronous actions on demand
 */
class RosInstinctActions : public Instinct::Actions
{
  public:
    /**
     * @param spinThread Spin up a separate thread for servicing actionlib requests
     */
    RosInstinctActions(bool spinThread, const std::string& actionServerName = "action_service");

    virtual unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue,
                                        const unsigned char bCheckForComplete) override;

    /**
     * @return Number of action requests made since creation
     */
    long getActionCount() const;

  protected:
    ros::NodeHandle rosHandle_;
    /**
     * Client for the actionlib server.
     * Used to make requests and check status
     */
    actionlib::SimpleActionClient<instinct_msgs::InstinctAction> actionClient_;
    /**
     * Number of actions requested in total since creation.
     */
    long actionCount_ = 0;
    /**
     * Publisher for action result messages for logging.
     * Published at /log/action using the InstinctActionRequest message.
     */
    ros::Publisher logPub_;
    instinct_msgs::InstinctActionRequest logMessage_;
};
