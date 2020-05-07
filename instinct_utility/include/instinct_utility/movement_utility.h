#pragma once
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

/**
 * Utility class for assisting with ROS movement tasks
 */
class MovementUtility
{
  public:
    /**
     * Container for Roll, Pitch and Yaw Euler anglers
     */
    struct RPY
    {
        double roll;
        double pitch;
        double yaw;
    };

    /**
     * Extract roll, pitch and yaw (Euler) from a Quaternion ROS message
     *
     *
     * @param quat Quaternion to convert
     * @return Euler angle representation
     */
    static inline RPY rpyFromQuaternion(geometry_msgs::Quaternion quat)
    {
        return rpyFromQuaternion(quat.x, quat.y, quat.z, quat.w);
    }

    /**
     * Convert a quaternion to a Roll, Pitch, Yaw Euler angle representation in radians
     * Yaw is from 0 - 2 PI
     *
     * @param x
     * @param y
     * @param z
     * @param w
     * @return  Roll, Pitch and Yaw (radians) extracted from the quaternion
     */
    static RPY rpyFromQuaternion(double x, double y, double z, double w)
    {
        tf2::Quaternion quatRot(x, y, z, w);
        tf2::Matrix3x3 rotMatrix(quatRot);
        RPY result;
        rotMatrix.getEulerYPR(result.yaw, result.pitch, result.roll);
        // Correct the yaw to be between 0 to 360 instead of -180 to 180
        if (result.yaw < 0)
        {
            result.yaw += 2 * M_PI;
        }
        return result;
    }
};
