#ifndef DRONE_OFFBOARD_CORE_SAFETY_ALTITUDE_SAFETY_HPP_
#define DRONE_OFFBOARD_CORE_SAFETY_ALTITUDE_SAFETY_HPP_

/**
 * @file altitude_safety.hpp
 * @brief Altitude safety checker (NO ROS DEPENDENCY)
 * 
 * Checks altitude limits and provides emergency descent command.
 */

#include "../types/drone_state.hpp"
#include "../types/nav_command.hpp"
#include "../types/common.hpp"

namespace drone_follow {
namespace core {

// Altitude safety limits
constexpr double MAX_ALTITUDE_AGL = 120.0;         // m above ground
constexpr double MAX_ALTITUDE_MSL = 800.0;         // m above sea level

/**
 * @brief Altitude safety checker
 */
class AltitudeSafety {
public:
    AltitudeSafety() = default;
    
    /**
     * @brief Check if altitude is within safe limits
     * @return true if safe, false if exceeded
     */
    bool is_safe(const DroneState& drone) const;
    
    /**
     * @brief Get emergency descent command
     * @return NavCommand for controlled descent
     */
    NavCommand get_emergency_descent_command() const;
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_SAFETY_ALTITUDE_SAFETY_HPP_
