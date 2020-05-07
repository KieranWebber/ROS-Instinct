//
// Created by kieranwebber on 09/03/2020.
//

#pragma once

#include <instinct/Instinct.h>

/**
 * Stub version of the InstinctMonitor used to simply log errors and node failures.
 * Used by default by the RosInstinctPlanner as it requires a monitor to be registered.
 */
class StubInstinctMonitor : public Instinct::Monitor
{
  public:
    virtual unsigned char nodeExecuted(const Instinct::PlanNode* pPlanNode) override
    {
        return 0;
    }
    virtual unsigned char nodeSuccess(const Instinct::PlanNode* pPlanNode) override
    {
        return 0;
    }
    virtual unsigned char nodeInProgress(const Instinct::PlanNode* pPlanNode) override
    {
        return 0;
    }
    virtual unsigned char nodeFail(const Instinct::PlanNode* pPlanNode) override
    {
        ROS_INFO("Node Fail! ID: %d", pPlanNode->sElement.sReferences.bRuntime_ElementID);
        return 0;
    }
    virtual unsigned char nodeError(const Instinct::PlanNode* pPlanNode) override
    {
        ROS_INFO("Node Error! ID: %d", pPlanNode->sElement.sReferences.bRuntime_ElementID);
        return 0;
    }
    virtual unsigned char nodeSense(const Instinct::ReleaserType* pReleaser, const int nSenseValue) override
    {
        return 0;
    }
};