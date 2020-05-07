#pragma once
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <instinct_msgs/InstinctAction.h>

/**
 * Extendable interface providing ROS-Instinct actions using a actionlib server.
 *
 * Actions are implemented by extending executeAction.
 * Service calls to executeActions are performed on a separate thread and are permitted to be blocking.
 * Long running actions should take care to query ros::ok and actionServer_.isPreemptRequested() to see if the task
 * should be aborted. The action server is automatically advertised on instantiation
 */
class InstinctActionService
{
  public:
    InstinctActionService(ros::NodeHandle& nh, const std::string& serviceName = "action_service");

  protected:
    /**
     * Callback used on an action request.
     * Runs on a dedicated thread allowing a action request to be blocking
     *
     * @param actionId Id of the action to execute
     * @param actionValue Parameter of the action (if applicable)
     * @return A Instinct Action status code
     */
    virtual unsigned char executeAction(const int actionId, const int actionValue) = 0;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<instinct_msgs::InstinctAction> actionServer_;
    /**
     * Publish feedback back to the planner node for consumption by the action client
     *
     * @param feedback Feedback status code
     */
    void publishFeedback(int feedback);

  private:
    instinct_msgs::InstinctActionFeedback feedback_;
    instinct_msgs::InstinctActionResult result_;
    /**
     * Callback wrapping executeAction that handles extracting params from the goal, performing the action
     * and returning the result
     *
     * @param goal Goal message the action request
     */
    void actionSeverRequest(const instinct_msgs::InstinctGoalConstPtr& goal);
    /**
     * Set the action status for reading by the client.
     * Called automatically by actionServerRequest after executing the action
     *
     * @param result Action result status - Sent to client
     */
    void setActionResult(unsigned char result);
};
