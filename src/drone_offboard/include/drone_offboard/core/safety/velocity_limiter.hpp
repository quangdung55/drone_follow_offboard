#ifndef DRONE_OFFBOARD_CORE_SAFETY_VELOCITY_LIMITER_HPP_
#define DRONE_OFFBOARD_CORE_SAFETY_VELOCITY_LIMITER_HPP_

/**
 * @file velocity_limiter.hpp
 * @brief Velocity safety limiter (NO ROS DEPENDENCY)
 * 
 * Ensures navigation commands stay within safe limits.
 */

#include <string>
#include "../types/common.hpp"

namespace drone_follow {
namespace core {

/**
 * @brief Result of velocity check
 */
struct VelocityCheckResult {
    bool passed;
    std::string reason;
};

/**
 * @brief Velocity safety limiter
 */
class VelocityLimiter {
public:
    VelocityLimiter() = default;
    
    /**
     * @brief Check if velocities are within safe limits
     * @return Result with pass/fail and reason
     */
    VelocityCheckResult check(double vel_forward, double vel_left, 
                             double vel_up, double yaw_rate) const;
    
    /**
     * @brief Clamp velocities to safe limits
     */
    void clamp(double& vel_forward, double& vel_left, 
               double& vel_up, double& yaw_rate) const;
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_SAFETY_VELOCITY_LIMITER_HPP_
