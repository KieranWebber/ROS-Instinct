#include "../include/ros_instinct_core/mapped_instinct_actions.h"
unsigned char MappedInstinctActions::executeAction(const Instinct::actionID nAction, const int nActionValue,
                                                   const unsigned char bCheckForComplete)
{
    auto actionInterface = getMatching(nAction);
    if (actionInterface == nullptr)
    {
        ROS_ERROR("Missing action mapping for action ID: %d", nAction);
        return INSTINCT_FAIL;
        ;
    }
    return actionInterface->second->executeAction(nAction - actionInterface->first.getStart(), nActionValue,
                                                  bCheckForComplete);
}
void MappedInstinctActions::fromRosParams(const std::map<std::string, std::string>& config)
{
    for (auto& mapItem : config)
    {
        IntRange range = MappingUtility::parseStringKey(mapItem.first);
        addItem(range, new RosInstinctActions(true, mapItem.second));
    }
}
MappedInstinctActions::MappedInstinctActions() : RangeBasedDemultiplexer()
{
}
MappedInstinctActions::~MappedInstinctActions()
{
    for (auto& d : rangeMap_)
    {
        delete d.second;
    }
}
