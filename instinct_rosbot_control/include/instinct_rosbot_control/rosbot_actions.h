#pragma once

/**
 * Action IDs for ROSBot control
 */
namespace rosbot_instinct
{
namespace actions
{
/**
 * Set the forward speed of the robot
 */
const int SET_SPEED = 1;
/**
 * Set the angular turn rate of the robot
 */
const int SET_TURN_RATE = 2;
/**
 * Turn a fixed angle in degrees
 */
const int TURN = 3;
/**
 * Turn a fixed angle in degrees into the most open direction.
 * Decided using the front ultrasound sensors.
 */
const int OPEN_TURN = 4;
}  // namespace actions
}  // namespace rosbot_instinct
