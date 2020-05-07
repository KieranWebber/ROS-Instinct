#pragma once

#include <instinct/Instinct.h>
#include <ros/duration.h>
class UnconnectedInstinctAction : public Instinct::Actions
{
    virtual unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue,
                                        const unsigned char bCheckForComplete) override
    {
        // ros::Duration(0.1).sleep();
        return INSTINCT_SUCCESS;
    }
};
