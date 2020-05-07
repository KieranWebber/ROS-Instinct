#include <ros/ros.h>
#include "ros_instinct_core/ros_instinct_actions.h"

unsigned char RosInstinctActions::executeAction(const Instinct::actionID nAction, const int nActionValue,
                                                const unsigned char bCheckForComplete)
{
    // Try to restore the connection
    // Stalling the planner no longer matters if we can't execute actions!
    if (!actionClient_.isServerConnected())
    {
        ROS_ERROR("Action Service disconnected! Waiting for reconnect");
        actionClient_.waitForServer(ros::Duration(0.2));
    }
    // Send a new goal only if not checking for completion
    if (!bCheckForComplete)
    {
        instinct_msgs::InstinctGoal goal;
        goal.action_id = nAction;
        goal.action_value = nActionValue;
        actionClient_.sendGoal(goal);
        actionCount_++;
    }
    // actionClient_.getState();
    actionlib::SimpleClientGoalState state =
            actionClient_.getState();  // actionClient_.sendGoalAndWait(goal, ros::Duration(2.0));
    // If the action has finished mark success
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Action ID: %d. Action Value: %d SUCCESS", nAction, nActionValue);
        logMessage_.action_id = nAction;
        logMessage_.action_value = nActionValue;
        logMessage_.action_success = true;
        logPub_.publish(logMessage_);
        return INSTINCT_SUCCESS;
    }
    // Pending or active means the action is unfinished
    else if (state == actionlib::SimpleClientGoalState::PENDING || state == actionlib::SimpleClientGoalState::ACTIVE)
    {
        // ROS_INFO("Action ID: %d. Action Value: %d Pending", nAction, nActionValue);
        return INSTINCT_IN_PROGRESS;
    }
    // Anything else is failure / cancellation
    else
    {
        ROS_INFO("Action ID: %d. Action Value: %d FAIL", nAction, nActionValue);
        logMessage_.action_id = nAction;
        logMessage_.action_value = nActionValue;
        logMessage_.action_success = false;
        logPub_.publish(logMessage_);
        return INSTINCT_FAIL;
    }
}
RosInstinctActions::RosInstinctActions(bool spinThread, const std::string& actionServerName)
  : actionClient_(actionServerName, spinThread)
{
    actionClient_.waitForServer();
    logPub_ = rosHandle_.advertise<instinct_msgs::InstinctActionRequest>("log/action", 0);
}

long RosInstinctActions::getActionCount() const
{
    return actionCount_;
}
