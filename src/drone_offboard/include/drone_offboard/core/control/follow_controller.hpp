#ifndef DRONE_OFFBOARD_CORE_CONTROL_FOLLOW_CONTROLLER_HPP_
#define DRONE_OFFBOARD_CORE_CONTROL_FOLLOW_CONTROLLER_HPP_

/**
 * @file follow_controller.hpp
 * @brief Follow controller using ArduPilot-style kinematic control (NO ROS DEPENDENCY)
 * 
 * Algorithm:
 * 1. Calculate position error in Map-ENU frame
 * 2. Apply sqrt controller for smooth position tracking
 * 3. Add target velocity feedforward
 * 4. Apply kinematic shaping (jerk/acceleration limiting)
 * 5. Integrate shaped acceleration to velocity
 * 6. Transform to drone body frame
 * 
 * Usage:
 *   FollowController controller(params);
 *   NavCommand cmd = controller.update(drone_state, target_state, dt);
 */

#include <Eigen/Dense>
#include "../types/drone_state.hpp"
#include "../types/target_state.hpp"
#include "../types/nav_command.hpp"
#include "../types/parameters.hpp"

namespace drone_follow {
namespace core {

/**
 * @brief Follow controller for GPS-based target tracking
 */
class FollowController {
public:
    /**
     * @brief Constructor
     * @param params Follow control parameters
     */
    explicit FollowController(const FollowControlParams& params);
    
    /**
     * @brief Update controller and compute navigation command
     * @param drone Current drone state
     * @param target Current target state
     * @param dt Time delta (seconds)
     * @return Navigation command in body frame
     */
    NavCommand update(const DroneState& drone, const TargetState& target, double dt);
    
    /**
     * @brief Reset internal state (shaped velocity/acceleration)
     */
    void reset();
    
    /**
     * @brief Get current parameters
     */
    const FollowControlParams& params() const { return params_; }
    
    /**
     * @brief Update parameters
     */
    void set_params(const FollowControlParams& params) { params_ = params; }
    
private:
    // Parameters
    FollowControlParams params_;
    
    // Shaped state (velocity and acceleration)
    Eigen::Vector2d shaped_vel_xy_;      // Horizontal velocity (m/s)
    Eigen::Vector2d shaped_accel_xy_;    // Horizontal acceleration (m/s²)
    double shaped_vel_z_;                 // Vertical velocity (m/s)
    double shaped_accel_z_;               // Vertical acceleration (m/s²)
    
    /**
     * @brief Calculate adaptive follow distance based on target speed
     */
    double calculate_follow_distance(double target_speed) const;
    
    /**
     * @brief Calculate offset in ENU frame
     */
    Eigen::Vector2d calculate_offset_enu(const TargetState& target, double desired_dist) const;
    
    /**
     * @brief Rotate 2D vector by angle
     */
    Eigen::Vector2d rotate_vector_2d(const Eigen::Vector2d& vec, double angle_rad) const;
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_CONTROL_FOLLOW_CONTROLLER_HPP_
