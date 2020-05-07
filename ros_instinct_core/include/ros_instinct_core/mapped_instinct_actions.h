#pragma once

#include <instinct/Instinct.h>
#include <ros/node_handle.h>
#include <vector>
#include "mapping_utility.h"
#include "ros_instinct_actions.h"
#include "range_demultiplexer.h"

/**
 * Composite action interface containing multiple ROSInstinctAction interfaces that are demuxed using the
 * actionId used by the planner. Useful for using multiple action nodes in parallel.
 *
 * Items can be added manually or through a ROS param map.
 *
 * Note: The action server used by RosInstinctActions cannot be copied and instead pointers to the interface are used
 */
class MappedInstinctActions : public RangeBasedDemultiplexer<RosInstinctActions*>, public Instinct::Actions
{
  public:
    MappedInstinctActions();
    virtual unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue,
                                        const unsigned char bCheckForComplete) override;
    virtual ~MappedInstinctActions();
    /**
     * Parse a range to node map taken from the ROS parameter server.
     * Key is in format: start-end
     * Value is in format: nodename
     *
     * @param config Map of integer ranges to sense node names
     */
    void fromRosParams(const std::map<std::string, std::string>& config);
};
