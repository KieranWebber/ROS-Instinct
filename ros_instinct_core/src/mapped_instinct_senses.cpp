#include "../include/ros_instinct_core/mapped_instinct_senses.h"
#include "ros_instinct_core/ros_instinct_senses.h"

int MappedInstinctSenses::readSense(const Instinct::senseID nSense)
{
    // As idToNodeMap is sorted binary search could be used to make this faster
    // In reality this is very cheap as we expect to have < 10 nodes
    auto mappedInterface = getMatching(nSense);
    if (mappedInterface == nullptr)
    {
        ROS_ERROR("Misconfigured sense range mapping! No mapping found for sense ID %d", nSense);
        return 0;
    }
    return mappedInterface->second.readSense(nSense - mappedInterface->first.getStart());
}

void MappedInstinctSenses::fromROSParams(const std::map<std::string, std::string>& config)
{
    for (auto& mapItem : config)
    {
        ROS_INFO("%s : %s", mapItem.first.c_str(), mapItem.second.c_str());
        IntRange range = MappingUtility::parseStringKey(mapItem.first);
        addItem(range, RosInstinctSenses(handle_, mapItem.second));
    }
}
MappedInstinctSenses::MappedInstinctSenses(const ros::NodeHandle& handle) : RangeBasedDemultiplexer(), handle_(handle)
{
}
