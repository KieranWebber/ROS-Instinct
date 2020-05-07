#pragma once  // SRC_SIMPLE_SENSE_H

#include <ros_instinct_sense/sense_service.h>

class SimpleSense : public SenseService
{
  public:
    SimpleSense(const ros::NodeHandle& handle);
    virtual int readSense(int senseId) override;
};
