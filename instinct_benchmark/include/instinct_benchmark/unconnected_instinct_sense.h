#pragma once  // ROS_INSTINCT_CORE_UNCONNECTED_INSTINCT_SENSE_H

#include <instinct/Instinct.h>
class UnconnectedInstinctSense : public Instinct::Senses
{
  public:
    virtual int readSense(const Instinct::senseID nSense) override
    {
        return rand() % 100;
    }
};
