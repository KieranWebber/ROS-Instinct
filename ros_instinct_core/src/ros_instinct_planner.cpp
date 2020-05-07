#include <ros_instinct_core/planner_utility.h>
#include "../include/ros_instinct_core/ros_instinct_planner.h"

StubInstinctMonitor RosInstinctPlanner::stubMonitor_ = StubInstinctMonitor();

unsigned char RosInstinctPlanner::runCycle()
{
    // Planner will be null if not initialised
    if (planner_ == nullptr)
    {
        throw "No loaded instinct plan!";
    }
    // Single cycle
    planner_->processTimers(1);
    return planner_->runPlan();
}
RosInstinctPlanner::RosInstinctPlanner(Instinct::Senses* senses, Instinct::Actions* actions)
  : senses_(senses), actions_(actions)
{
}
bool RosInstinctPlanner::loadPlan(const char* plan, Instinct::Monitor* monitor)
{
    // Setup a new planner and load the plan from text
    names_ = new Instinct::Names(INSTINCT_NAMES_BUFFER_SIZE);
    // 6 node types - see Instinct for details
    Instinct::instinctID nPlanSize[INSTINCT_NODE_TYPES] = { 0, 0, 0, 0, 0, 0 };
    planner_ = new Instinct::CmdPlanner(nPlanSize, senses_, actions_, monitor);
    PlannerUtility::loadPlan(planner_, names_, plan);
    return true;
}
bool RosInstinctPlanner::loadPlan(const char* plan)
{
    return loadPlan(plan, &stubMonitor_);
}
Instinct::CmdPlanner* RosInstinctPlanner::getPlanner()
{
    return planner_;
}
Instinct::Names* RosInstinctPlanner::getNames()
{
    return names_;
}
Instinct::Senses* RosInstinctPlanner::getSenseInterface()
{
    return senses_;
}
Instinct::Actions* RosInstinctPlanner::getActionInterface()
{
    return actions_;
}
