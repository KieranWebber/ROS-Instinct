#pragma once

#include <instinct/Instinct.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "ros_instinct_core/mapping_utility.h"
#include "ros_instinct_core/ros_instinct_senses.h"
#include "ros_instinct_core/range_demultiplexer.h"

/**
 * Composite sense interface containing multiple ROSInstinctSense interfaces that are demuxed using the
 * senseId used by resense. Useful for using multiple sense nodes in parallel.
 *
 * Items can be added manually or through a ROS param map.
 */
class MappedInstinctSenses : public RangeBasedDemultiplexer<RosInstinctSenses>, public Instinct::Senses
{
  public:
    MappedInstinctSenses(const ros::NodeHandle& handle);
    virtual int readSense(const Instinct::senseID nSense) override;
    /**
     * Parse a range to node map taken from the ROS paramter server.
     * Key is in format: start-end
     * Value is in format: nodename
     *
     * @param config Map of integer ranges to sense node names
     */
    void fromROSParams(const std::map<std::string, std::string>& config);

  protected:
    ros::NodeHandle handle_;
};
