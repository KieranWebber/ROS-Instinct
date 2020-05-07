#pragma once  // ROS_INSTINCT_CORE_SIMPLE_ACTION_H

#include <ros_instinct_actions/instinct_action_service.h>
class SimpleAction : public InstinctActionService
{
  protected:
  public:
    SimpleAction(ros::NodeHandle& nh);

  protected:
    virtual unsigned char executeAction(const int actionId, const int actionValue) override;
};
