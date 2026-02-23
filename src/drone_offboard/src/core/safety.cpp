/**
 * @file safety.cpp
 * @brief Safety validators implementation
 */

#include "drone_offboard/core/safety/estimate_validator.hpp"
#include "drone_offboard/core/safety/velocity_limiter.hpp"
#include "drone_offboard/core/safety/altitude_safety.hpp"
#include <cmath>
#include <algorithm>

namespace drone_follow {
namespace core {

// ============================================================================
// ESTIMATE VALIDATOR
// ============================================================================

EstimateValidator::EstimateValidator(const EstimatorParams& params)
    : params_(params) {}

bool EstimateValidator::is_error_too_large(const TargetState& current_state,
                                          const Eigen::Vector3d& last_estimate_pos,
                                          const Eigen::Vector3d& last_estimate_vel) const {
    if (!current_state.valid) {
        return false;
    }
    
    // Skip check if no previous estimate
    if (last_estimate_pos.norm() < 0.1) {
        return false;
    }
    
    const double timeout_sec = params_.timeout_sec;
    
    // Calculate thresholds from kinematic limits
    const double pos_thresh_horiz_m = 2.5 * std::pow(timeout_sec * 0.5, 2);
    const double pos_thresh_vert_m = 1.0 * std::pow(timeout_sec * 0.5, 2);
    
    const double vel_thresh_horiz_ms = calc_max_velocity_change(2.5, 5.0, timeout_sec);
    const double vel_thresh_vert_ms = calc_max_velocity_change(1.0, 2.0, timeout_sec);
    
    // Calculate errors
    Eigen::Vector3d pos_error = last_estimate_pos - current_state.pos_enu;
    Eigen::Vector3d vel_error = last_estimate_vel - current_state.vel_enu;
    
    double pos_error_horiz = pos_error.head<2>().norm();
    double vel_error_horiz = vel_error.head<2>().norm();
    double pos_error_vert = std::abs(pos_error.z());
    double vel_error_vert = std::abs(vel_error.z());
    
    bool pos_horiz_bad = pos_error_horiz > pos_thresh_horiz_m;
    bool vel_horiz_bad = vel_error_horiz > vel_thresh_horiz_ms;
    bool pos_vert_bad = pos_error_vert > pos_thresh_vert_m;
    bool vel_vert_bad = vel_error_vert > vel_thresh_vert_ms;
    
    // Check for unreasonable values
    double target_vel_horiz = current_state.vel_enu.head<2>().norm();
    double target_accel_horiz = current_state.accel_enu.head<2>().norm();
    double target_accel_vert = std::abs(current_state.accel_enu.z());
    
    bool velocity_unreasonable = target_vel_horiz > MAX_TARGET_VELOCITY;
    bool accel_unreasonable = (target_accel_horiz > MAX_TARGET_ACCELERATION) ||
                             (target_accel_vert > MAX_TARGET_ACCELERATION);
    
    return pos_horiz_bad || vel_horiz_bad || pos_vert_bad || vel_vert_bad ||
           velocity_unreasonable || accel_unreasonable;
}

double EstimateValidator::calc_max_velocity_change(double accel_max, double jerk_max,
                                                   double timeout_sec) const {
    const double t_jerk = accel_max / jerk_max;
    const double t_total_jerk = 2.0 * t_jerk;
    
    if (timeout_sec >= t_total_jerk) {
        const double t_const = timeout_sec - t_total_jerk;
        const double delta_v_jerk = 0.5 * accel_max * t_jerk;
        const double delta_v_const = accel_max * t_const;
        return 2.0 * delta_v_jerk + delta_v_const;
    } else {
        const double t_half = timeout_sec * 0.5;
        return 0.5 * jerk_max * std::pow(t_half, 2);
    }
}

// ============================================================================
// VELOCITY LIMITER
// ============================================================================

VelocityCheckResult VelocityLimiter::check(double vel_forward, double vel_left,
                                          double vel_up, double yaw_rate) const {
    VelocityCheckResult result;
    result.passed = true;
    
    if (std::abs(vel_forward) > CMD_VEL_FORWARD_MAX) {
        result.passed = false;
        result.reason = "Forward velocity exceeded";
        return result;
    }
    
    if (std::abs(vel_left) > CMD_VEL_LATERAL_MAX) {
        result.passed = false;
        result.reason = "Lateral velocity exceeded";
        return result;
    }
    
    if (std::abs(vel_up) > CMD_VEL_VERTICAL_MAX) {
        result.passed = false;
        result.reason = "Vertical velocity exceeded";
        return result;
    }
    
    if (std::abs(yaw_rate) > CMD_YAW_RATE_MAX) {
        result.passed = false;
        result.reason = "Yaw rate exceeded";
        return result;
    }
    
    return result;
}

void VelocityLimiter::clamp(double& vel_forward, double& vel_left,
                           double& vel_up, double& yaw_rate) const {
    vel_forward = std::clamp(vel_forward, -CMD_VEL_FORWARD_MAX, CMD_VEL_FORWARD_MAX);
    vel_left = std::clamp(vel_left, -CMD_VEL_LATERAL_MAX, CMD_VEL_LATERAL_MAX);
    vel_up = std::clamp(vel_up, -CMD_VEL_VERTICAL_MAX, CMD_VEL_VERTICAL_MAX);
    yaw_rate = std::clamp(yaw_rate, -CMD_YAW_RATE_MAX, CMD_YAW_RATE_MAX);
}

// ============================================================================
// ALTITUDE SAFETY
// ============================================================================

bool AltitudeSafety::is_safe(const DroneState& drone) const {
    // Check relative altitude (AGL)
    if (drone.alt_rel > MAX_ALTITUDE_AGL) {
        return false;
    }
    
    // Check absolute altitude (MSL)
    if (drone.alt_msl > MAX_ALTITUDE_MSL) {
        return false;
    }
    
    return true;
}

NavCommand AltitudeSafety::get_emergency_descent_command() const {
    NavCommand cmd;
    cmd.vel_forward = 0.0;
    cmd.vel_left = 0.0;
    cmd.vel_up = -CMD_VEL_VERTICAL_MAX;  // Descend at max rate
    cmd.yaw_rate = 0.0;
    return cmd;
}

} // namespace core
} // namespace drone_follow
