#include <ros_instinct_core/ros_instinct_planner.h>
#include <ros_instinct_core/ros_instinct_senses.h>
#include <ros_instinct_core/ros_instinct_actions.h>
#include <ros_instinct_core/planner_utility.h>
#include "ros_instinct_core/mapped_instinct_senses.h"

#include <fstream>
#include <ros_instinct_core/mapped_instinct_actions.h>

std::string loadFileToString(const char* name)
{
    std::ifstream in(name);
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return contents;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "instinct_planner");
    ros::NodeHandle n;
    ros::NodeHandle privateHandle("~");
    std::string instinctPlan;
    privateHandle.param<std::string>("instinct_plan", instinctPlan, "");
    if (instinctPlan.empty())
    {
        privateHandle.param<std::string>("instinct_plan_file", instinctPlan,
                                         "/home/kieranwebber/catkin_ws/src/ros_instinct_core/plans/"
                                         "OnOffPlan_with_store.inst");
        if (!instinctPlan.empty())
        {
            // Load using path name
            ROS_INFO("Reading plan from disk at %s", instinctPlan.c_str());
            instinctPlan = loadFileToString(instinctPlan.c_str());
        }
        else
        {
            ROS_INFO("No Instinct plan found! Aborting!");
            return 1;
        }
    }
    else
    {
        ROS_INFO("Reading plan from parameter server!");
    }

    // TODO - Logic for sense + action interface creation could be rolled together
    ROS_INFO("Connecting to ROS sense interface(s)");
    Instinct::Senses* senseInterface;
    // Load a demultiplexed configuration from ROS parameters
    std::map<std::string, std::string> senseMap;
    privateHandle.getParam("instinct_sense_map", senseMap);
    ROS_INFO("%d", (int)senseMap.size());
    if (!senseMap.empty())
    {
        ROS_INFO("Creating %d interfaces based on %d mappings", (int)senseMap.size(), (int)senseMap.size());
        MappedInstinctSenses* mappedSenseInterface = new MappedInstinctSenses(n);
        mappedSenseInterface->fromROSParams(senseMap);
        senseInterface = mappedSenseInterface;
    }
    else
    {
        ROS_INFO("Using default sense interface!");
        senseInterface = new RosInstinctSenses(n);
    }

    ROS_INFO("Connecting to ROS action interface(s)");
    Instinct::Actions* actionInterface;
    // Load a demultiplexed configuration from ROS parameters
    std::map<std::string, std::string> actionMap;
    // senseMap.insert(std::make_pair("0-20", "store_sense_service"));
    privateHandle.getParam("instinct_action_map", actionMap);
    // Was a configuration found? If not use the default
    if (!actionMap.empty())
    {
        ROS_INFO("Creating %d interfaces based on %d mappings", (int)actionMap.size(), (int)actionMap.size());
        MappedInstinctActions* mappedActionInterface = new MappedInstinctActions();
        mappedActionInterface->fromRosParams(actionMap);
        actionInterface = mappedActionInterface;
    }
    else
    {
        // Default client
        // Run action service client in a separate thread so avoid blocking the planner
        ROS_INFO("Using default action interface!");
        actionInterface = new RosInstinctActions(true);
    }

    // Get the planner frequency from the parameter server
    int spinRate;
    privateHandle.param<int>("instinct_spin_rate", spinRate, 100);

    ros::Rate loop_rate(spinRate);
    ROS_INFO("Parsing plan into planner");
    RosInstinctPlanner planner(senseInterface, actionInterface);
    planner.loadPlan(instinctPlan.c_str());

    ROS_INFO("Planner Ready! Starting planning cycles at %dHz", spinRate);
    while (ros::ok())
    {
        // Spin ROS
        ros::spinOnce();
        // Spin the planner for a single cycle
        planner.runCycle();
        // Sleep at the fixed interval
        loop_rate.sleep();
    }
}