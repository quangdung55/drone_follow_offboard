#ifndef DRONE_OFFBOARD_CORE_CONTROL_YAW_CONTROLLER_HPP_
#define DRONE_OFFBOARD_CORE_CONTROL_YAW_CONTROLLER_HPP_

/**
 * @file yaw_controller.hpp
 * @brief Yaw controller with gimbal tracking and heading follow (NO ROS DEPENDENCY)
 * 
 * Two control modes blended together:
 * 1. Gimbal Tracking (Reactive): Keep target centered in camera view
 * 2. Heading Follow (Predictive): Smoothly track target's heading
 * 
 * Usage:
 *   YawController controller(params);
 *   
 *   // Update with gimbal measurement (optional)
 *   GimbalMeasurement gimbal;
 *   gimbal.pan_error_deg = -10.0;
 *   gimbal.timestamp_sec = current_time;
 *   controller.update_gimbal(gimbal);
 *   
 *   // Calculate yaw rate
 *   double yaw_rate = controller.update(drone_state, target_state, dt);
 */

#include "../types/drone_state.hpp"
#include "../types/target_state.hpp"
#include "../types/common.hpp"
#include "../types/parameters.hpp"
#include <optional>

namespace drone_follow {
namespace core {

/**
 * @brief Gimbal tracker with timeout
 */
class GimbalTracker {
public:
    GimbalTracker() : timeout_sec_(0.5) {}
    
    /**
     * @brief Update gimbal measurement
     */
    void update(double pan_error_deg, double timestamp_sec);
    
    /**
     * @brief Get gimbal error (with timeout check)
     * @return true if valid data available
     */
    bool get_error(double& pan_error_deg, double current_time_sec) const;
    
private:
    double pan_error_deg_ = 0.0;
    double last_update_sec_ = 0.0;
    double timeout_sec_;
};

/**
 * @brief Yaw controller for target tracking
 */
class YawController {
public:
    /**
     * @brief Constructor
     * @param params Yaw control parameters
     */
    explicit YawController(const YawControlParams& params);
    
    /**
     * @brief Update gimbal measurement
     */
    void update_gimbal(const GimbalMeasurement& meas);
    
    /**
     * @brief Calculate yaw rate command
     * @param drone Current drone state
     * @param target Current target state
     * @param dt Time delta (seconds)
     * @param current_time Current time (seconds since epoch)
     * @return Yaw rate command (rad/s) with kinematic shaping
     */
    double update(const DroneState& drone, const TargetState& target, 
                  double dt, double current_time_sec);
    
    /**
     * @brief Reset internal state
     */
    void reset();
    
    /**
     * @brief Get current parameters
     */
    const YawControlParams& params() const { return params_; }
    
    /**
     * @brief Update parameters
     */
    void set_params(const YawControlParams& params) { params_ = params; }
    
private:
    // Parameters
    YawControlParams params_;
    
    // Gimbal tracking
    GimbalTracker gimbal_tracker_;
    
    // Shaped state
    double shaped_yaw_rate_;      // Shaped yaw rate (rad/s)
    double shaped_yaw_accel_;     // Shaped yaw acceleration (rad/s²)
    
    /**
     * @brief Calculate gimbal-based yaw rate
     */
    double calculate_gimbal_yaw_rate(double current_time_sec);
    
    /**
     * @brief Calculate heading-follow yaw rate
     */
    double calculate_heading_yaw_rate(const DroneState& drone, const TargetState& target);
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_CONTROL_YAW_CONTROLLER_HPP_
