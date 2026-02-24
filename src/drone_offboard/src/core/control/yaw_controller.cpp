/**
 * @file yaw_controller.cpp
 * @brief Yaw controller implementation with gimbal tracking and heading follow
 */

#include "drone_offboard/core/control/yaw_controller.hpp"
#include "drone_offboard/core/control/ap_control.hpp"
#include <cmath>

namespace drone_follow {
namespace core {

// ============================================================================
// GIMBAL TRACKER
// ============================================================================

void GimbalTracker::update(double pan_error_deg, double timestamp_sec) {
    pan_error_deg_ = pan_error_deg;
    last_update_sec_ = timestamp_sec;
}

bool GimbalTracker::get_error(double& pan_error_deg, double current_time_sec) const {
    double time_since_update = current_time_sec - last_update_sec_;
    if (time_since_update > timeout_sec_ || last_update_sec_ == 0.0) {
        return false;
    }
    pan_error_deg = pan_error_deg_;
    return true;
}

// ============================================================================
// YAW CONTROLLER
// ============================================================================

YawController::YawController(const YawControlParams& params)
    : params_(params),
      shaped_yaw_rate_(0.0),
      shaped_yaw_accel_(0.0) {}

void YawController::update_gimbal(const GimbalMeasurement& meas) {
    gimbal_tracker_.update(meas.pan_error_deg, meas.timestamp_sec);
}

double YawController::update(const DroneState& drone, const TargetState& target,
                            double dt, double current_time_sec) {
    // Calculate gimbal-based yaw rate
    double yaw_rate_gimbal = calculate_gimbal_yaw_rate(current_time_sec);
    
    // Calculate heading-follow yaw rate
    double yaw_rate_heading = 0.0;
    if (params_.heading_blend_enable && target.valid) {
        yaw_rate_heading = calculate_heading_yaw_rate(drone, target);
    }
    
    // Blend the two modes
    double blend_weight = params_.heading_blend_enable ? params_.heading_blend_weight : 0.0;
    double yaw_rate_target = (1.0 - blend_weight) * yaw_rate_gimbal + 
                            blend_weight * yaw_rate_heading;
    
    // Apply jerk limiting (kinematic shaping)
    if (dt > CONTROL_LOOP_DT_MIN && dt < CONTROL_LOOP_DT_MAX) {
        ap_control::shape_vel_accel(
            yaw_rate_target,
            0.0,                        // No accel feedforward
            shaped_yaw_rate_,
            shaped_yaw_accel_,
            -params_.accel_max_h,
            params_.accel_max_h,
            params_.jerk_max_h,
            dt,
            true
        );
        // NOTE: Don't integrate acceleration here - we're shaping rate directly
    }
    
    return shaped_yaw_rate_;
}

void YawController::reset() {
    shaped_yaw_rate_ = 0.0;
    shaped_yaw_accel_ = 0.0;
}

double YawController::calculate_gimbal_yaw_rate(double current_time_sec) {
    double pan_error_deg;
    if (gimbal_tracker_.get_error(pan_error_deg, current_time_sec)) {
        // Apply deadzone
        if (std::abs(pan_error_deg) > params_.gimbal_deadzone) {
            // P controller (convert deg to rad)
            return -params_.kp_yaw * pan_error_deg * DEG_TO_RAD;
        }
    }
    return 0.0;
}

double YawController::calculate_heading_yaw_rate(const DroneState& drone, 
                                                 const TargetState& target) {
    // Only follow heading if target is moving
    double target_speed = target.vel_enu.head<2>().norm();
    if (target_speed <= SPEED_THRESHOLD_OFFSET) {
        return 0.0;
    }
    
    // Calculate heading error
    double heading_error = target.heading_rad - drone.yaw;
    
    // Wrap to [-PI, PI]
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
    
    // P controller
    return params_.heading_follow_kp * heading_error;
}

} // namespace core
} // namespace drone_follow
