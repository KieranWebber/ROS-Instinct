//
// Created by kieranwebber on 1/30/20.
//

#pragma once

#include "instinct/Instinct.h"
#include "ros_instinct_senses.h"
#include "ros_instinct_actions.h"
#include "ros_instinct_monitor.h"

#define INSTINCT_NAMES_BUFFER_SIZE 2000

/**
 * Wrapper around the Instinct Planner that uses ROS enabled links for senses/actions.
 */
class RosInstinctPlanner
{
  private:
    Instinct::CmdPlanner* planner_ = nullptr;
    Instinct::Names* names_ = nullptr;
    Instinct::Senses* senses_ = nullptr;
    Instinct::Actions* actions_ = nullptr;

    /**
     * A static instance of the stub monitor as it is stateless so can be shared between instances
     */
    static StubInstinctMonitor stubMonitor_;

  public:
    RosInstinctPlanner(Instinct::Senses* senses, Instinct::Actions* actions);
    /**
     * Load a new plan into the Instinct Planner with no plan monitor.
     *
     * @param plan Newline separated instinct plan in plain text
     * @return True if the plan was loaded without error
     */
    bool loadPlan(const char* plan);
    /**
     * Load a new plan into the Instinct Planner using the specified plan monitor.
     *
     * @param plan Newline separated instinct plan in plain text
     * @param monitor Plan monitor to register with the planner
     * @return True if the plan was loaded without error
     */
    bool loadPlan(const char* plan, Instinct::Monitor* monitor);
    /**
     * Run a single cycle of planning for both the planners and integrated timers
     *
     * @return Cycle status code
     */
    unsigned char runCycle();
    Instinct::CmdPlanner* getPlanner();
    Instinct::Names* getNames();
    Instinct::Senses* getSenseInterface();
    Instinct::Actions* getActionInterface();
};