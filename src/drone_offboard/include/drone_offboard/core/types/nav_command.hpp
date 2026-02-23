#ifndef DRONE_OFFBOARD_CORE_TYPES_NAV_COMMAND_HPP_
#define DRONE_OFFBOARD_CORE_TYPES_NAV_COMMAND_HPP_

/**
 * @file nav_command.hpp
 * @brief Pure C++ navigation command types (NO ROS DEPENDENCY)
 */

namespace drone_follow {
namespace core {

/**
 * @brief Navigation command in drone body frame
 * 
 * Body frame convention:
 * - Forward: +X (nose direction)
 * - Left: +Y (left wing)
 * - Up: +Z (top)
 * - Yaw rate: +Z rotation (counter-clockwise)
 */
struct NavCommand {
    double vel_forward;  // Forward velocity (Body Frame, m/s)
    double vel_left;     // Lateral velocity (Body Frame, m/s)
    double vel_up;       // Vertical velocity (Body Frame, m/s)
    double yaw_rate;     // Rotation rate (rad/s)
    
    /**
     * @brief Default constructor - all zeros (stop command)
     */
    NavCommand() 
        : vel_forward(0.0), vel_left(0.0), vel_up(0.0), yaw_rate(0.0) {}
    
    /**
     * @brief Constructor with values
     */
    NavCommand(double fwd, double left, double up, double yaw)
        : vel_forward(fwd), vel_left(left), vel_up(up), yaw_rate(yaw) {}
    
    /**
     * @brief Set all to zero (stop command)
     */
    void stop() {
        vel_forward = 0.0;
        vel_left = 0.0;
        vel_up = 0.0;
        yaw_rate = 0.0;
    }
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_TYPES_NAV_COMMAND_HPP_
