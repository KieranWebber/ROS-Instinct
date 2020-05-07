//
// Created by kieranwebber on 09/03/2020.
//

#pragma

/**
 * Sense IDs for the Instinct planner
 */
namespace rosbot_instinct
{
namespace senses
{
// Range sensors
/**
 * Front left ultrasound sensor.
 * Max distance: 9000mm.
 */
const int FRONT_LEFT_RANGE = 1;
/**
 * Front right ultrasound sensor.
 * Max distance: 9000mm.
 */
const int FRONT_RIGHT_RANGE = 2;
/**
 * Back left ultrasound sensor.
 * Max distance: 9000mm.
 */
const int BACK_LEFT_RANGE = 3;
/**
 * Back right ultrasound sensor.
 * Max distance: 9000mm.
 */
const int BACK_RIGHT_RANGE = 4;
/**
 * Minimum of the front ultrasound sensors.
 */
const int MIN_FRONT_RANGE = 5;
/**
 * Minimum of the back ultrasound sensors
 */
const int MIN_BACK_RANGE = 6;
// IMU sensors
/**
 * Roll angle (degrees)
 */
const int IMU_ROLL = 7;
/**
 * Pitch angle (degrees)
 */
const int IMU_PITCH = 8;
/**
 * Yaw angle (degrees)
 */
const int IMU_YAW = 9;
// Misc
/**
 * Current reported battery voltage (mV)
 */
const int BATTERY_VOLTAGE = 10;
/**
 * Current reported battery current (mA)
 */
const int BATTERY_CURRENT = 11;
}  // namespace senses
}  // namespace rosbot_instinct
