#pragma once

/**
 * Action status definitions as used by the Instinct planner.
 * Used when returning action status from the ActionServer
 */
namespace ros_instinct
{
namespace action
{
const int INSTINCT_FAIL = 0;
const int INSTINCT_SUCCESS = 1;
const int INSTINCT_IN_PROGRESS = 2;
const int INSTINCT_ERROR = 3;
}  // namespace action
}  // namespace ros_instinct
