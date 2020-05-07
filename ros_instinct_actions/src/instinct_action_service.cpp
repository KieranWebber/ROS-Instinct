#include "ros_instinct_actions/instinct_action_service.h"
#include "ros_instinct_actions/instinct_action_status.h"

// Perform the binding to link the class method as the callback for the actionserver as part of construction
InstinctActionService::InstinctActionService(ros::NodeHandle& nh, const std::string& serviceName)
  : nh_(nh), actionServer_(nh, serviceName, boost::bind(&InstinctActionService::actionSeverRequest, this, _1), false)
{
    // Start the server on creation
    actionServer_.start();
}

void InstinctActionService::actionSeverRequest(const instinct_msgs::InstinctGoalConstPtr& goal)
{
    ROS_INFO("ACTION %d, VALUE %d", goal->action_id, goal->action_value);
    unsigned char result = this->executeAction(goal->action_id, goal->action_value);
    setActionResult(result);
}

void InstinctActionService::setActionResult(unsigned char result)
{
    result_.result.action_response = result;
    if (result == ros_instinct::action::INSTINCT_FAIL || result == ros_instinct::action::INSTINCT_ERROR)
    {
        actionServer_.setAborted(result_.result);
    }
    else
    {
        actionServer_.setSucceeded(result_.result);
    }
}

void InstinctActionService::publishFeedback(int feedback)
{
    feedback_.feedback.action_feedback = feedback;
    actionServer_.publishFeedback(feedback_.feedback);
}
