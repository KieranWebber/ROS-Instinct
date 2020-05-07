#include <ros/ros.h>
#include <ros_instinct_core/ros_instinct_senses.h>
#include <boost/timer/timer.hpp>
#include <ros_instinct_core/ros_instinct_actions.h>
#include <ros_instinct_core/ros_instinct_planner.h>
#include <fstream>
#include <ros_instinct_core/planner_utility.h>
#include <ros_instinct_core/mapped_instinct_senses.h>
#include <instinct_store/instinct_store_service.h>

#include "../include/instinct_benchmark/unconnected_instinct_action.h"
#include "../include/instinct_benchmark/unconnected_instinct_sense.h"

std::string loadFileToString(const char* name)
{
    std::ifstream in(name);
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return contents;
}

double timerToMs(boost::timer::cpu_timer& timer)
{
    return (timer.elapsed().wall / 1000000.0);
}

void benchmarkKVStore()
{
    ros::NodeHandle handle;
    int senseCount = 1000;
    RosInstinctSenses senseHandle(handle, "store_sense_service");
    boost::timer::cpu_timer timer;
    for (int i = 0; i < senseCount; ++i)
    {
        senseHandle.readSense(rand());
    }
    timer.stop();
    ROS_INFO("Total Time (Read Store) [%f]ms", timerToMs(timer) / senseCount);
}

void benchmarkSenseInterface()
{
    ros::NodeHandle n;
    RosInstinctSenses* sense_interface = new RosInstinctSenses(n);
    int trialCount = 1000;
    boost::timer::cpu_timer timer;
    for (int j = 0; j < trialCount; ++j)
    {
        sense_interface->readSense(j);
    }
    timer.stop();
    double timePerReading = timerToMs(timer) / trialCount;
    ROS_INFO("Time per sense [%f]ms", timePerReading);
}

void benchmarkActionInterface(int trialCount)
{
    RosInstinctActions actionInterface(true);
    boost::timer::cpu_timer timer2;
    bool checkComplete = false;
    int sleepMs = 1000;
    for (int j = 0; j < trialCount; ++j)
    {
        while (actionInterface.executeAction(j, sleepMs, checkComplete) != INSTINCT_SUCCESS)
        {
            checkComplete = true;
            ros::Duration(0.0001).sleep();
        };
        checkComplete = false;
    }
    timer2.stop();
    double timePerReading2 = timerToMs(timer2) / trialCount;
    ROS_INFO("Time per action [%f]ms", timePerReading2);
}

void benchmarkPlanner(int cycleCount)
{
    auto plan = loadFileToString("/home/kieranwebber/catkin_ws/src/ros_instinct_core/plans/bench_plan.inst");
    ros::NodeHandle n;
    RosInstinctSenses senseBindings(n);
    RosInstinctActions actBindings(true);
    RosInstinctPlanner planner(&senseBindings, &actBindings);
    planner.loadPlan(plan.c_str());
    boost::timer::cpu_timer timer3;
    for (int k = 0; k < cycleCount; ++k)
    {
        planner.runCycle();
        ros::spinOnce();
    }
    timer3.stop();
    double timePerCycle = timerToMs(timer3) / cycleCount;
    ROS_INFO("Total Time (ROS PLANNER) [%f]ms", timerToMs(timer3));
    ROS_INFO("Time per cycle (ROS PLANNER) [%f]ms", timePerCycle);
    ROS_INFO("Sense Count: %d, Action Count %d", (int)senseBindings.getSenseCount(), (int)actBindings.getActionCount());
}

void benchmarkOriginalPlanner(int cycleCount)
{
    auto plan = loadFileToString("/home/kieranwebber/catkin_ws/src/ros_instinct_core/plans/bench_plan.inst");
    UnconnectedInstinctSense realSense;
    UnconnectedInstinctAction realAction;
    auto names = new Instinct::Names(INSTINCT_NAMES_BUFFER_SIZE);
    Instinct::instinctID nPlanSize[INSTINCT_NODE_TYPES] = { 0, 0, 0, 0, 0, 0 };
    StubInstinctMonitor monitor;
    auto originalPlanner = new Instinct::CmdPlanner(nPlanSize, &realSense, &realAction, &monitor);
    PlannerUtility::loadPlan(originalPlanner, names, plan.c_str());
    boost::timer::cpu_timer timer4;
    for (int k = 0; k < cycleCount; ++k)
    {
        originalPlanner->processTimers(1);
        originalPlanner->runPlan();
    }
    timer4.stop();
    double timePerOriginalCycle = timerToMs(timer4) / cycleCount;
    ROS_INFO("Total Time (ROS PLANNER) [%f]ms", timerToMs(timer4));
    ROS_INFO("Time per cycle (ORIGINAL PLANNER) [%f]ms", timePerOriginalCycle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "instinct_planner");
    ros::NodeHandle n;

    // Uncomment a specifc benchmark to perform.
    // Some benchmarks require additional changes to the action / sense nodes to function as intended
    // The benchmark action and sense nodes need started prior to running

    //    benchmarkSenseInterface();
    //    benchmarkKVStore();
    //    benchmarkActionInterface(1000);
    //    benchmarkOriginalPlanner(10000);
    //    benchmarkPlanner(10000);
    return 0;
}