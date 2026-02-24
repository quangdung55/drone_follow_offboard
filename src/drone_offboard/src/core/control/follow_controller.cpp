/**
 * @file follow_controller.cpp
 * @brief Follow controller implementation using ArduPilot-style kinematic control
 */

#include "drone_offboard/core/control/follow_controller.hpp"
#include "drone_offboard/core/control/ap_control.hpp"
#include <algorithm>
#include <cmath>

namespace drone_follow {
namespace core {

FollowController::FollowController(const FollowControlParams& params)
    : params_(params),
      shaped_vel_xy_(Eigen::Vector2d::Zero()),
      shaped_accel_xy_(Eigen::Vector2d::Zero()),
      shaped_vel_z_(0.0),
      shaped_accel_z_(0.0) {}

NavCommand FollowController::update(const DroneState& drone, const TargetState& target, double dt) {
    NavCommand cmd;
    
    // =========================================================================
    // 0. DT SANITY (AP style)
    // =========================================================================
    if (dt < 0.002 || dt > 0.1) {
        dt = 0.02; // fallback 50Hz
    }
    
    // =========================================================================
    // 1. POSITION ERROR (ENU)
    // =========================================================================
    Eigen::Vector2d pos_error_xy = target.pos_enu.head<2>() - drone.pos_enu.head<2>();
    double err_mag = pos_error_xy.norm();
    
    // =========================================================================
    // 2. RESET SAFEGUARD (target jump / re-acquire)
    // =========================================================================
    const double RESET_RADIUS = 50.0;  // meters
    if (err_mag > RESET_RADIUS) {
        shaped_vel_xy_.setZero();
        shaped_accel_xy_.setZero();
    }
    
    // =========================================================================
    // 3. DESIRED VELOCITY FROM POSITION (AP sqrt controller)
    // =========================================================================
    Eigen::Vector2d vel_pos_target{0.0, 0.0};
    
    if (err_mag > 1e-3) {
        double vel_mag = ap_control::sqrt_controller(
            err_mag,                     // Position error magnitude
            params_.kp_pos,              // Position P gain
            CMD_VEL_FORWARD_MAX,         // VELOCITY limit
            dt);
        
        vel_pos_target = pos_error_xy.normalized() * vel_mag;
    }
    
    // =========================================================================
    // 4. TARGET VELOCITY FEEDFORWARD (AP_Follow logic)
    // =========================================================================
    Eigen::Vector2d vel_ff{0.0, 0.0};
    
    double target_speed = target.vel_enu.head<2>().norm();
    double ff_scale = 0.0;
    
    if (target_speed > 0.2 && err_mag > 0.5) {
        double alignment = pos_error_xy.dot(target.vel_enu.head<2>()) / 
                          (err_mag * target_speed);
        
        ff_scale = std::clamp(alignment, 0.0, 1.0);
        
        // Reduce FF when target is braking
        if (target.vel_enu.head<2>().dot(target.accel_enu.head<2>()) < 0.0) {
            ff_scale *= 0.5;
        }
        
        vel_ff = target.vel_enu.head<2>() * ff_scale;
    }
    
    // =========================================================================
    // 5. TOTAL VELOCITY TARGET
    // =========================================================================
    Eigen::Vector2d vel_target_xy = vel_pos_target + vel_ff;
    
    // Clamp to safety limit
    const double VEL_MAX = CMD_VEL_FORWARD_MAX;
    if (vel_target_xy.norm() > VEL_MAX) {
        vel_target_xy = vel_target_xy.normalized() * VEL_MAX;
    }
    
    // =========================================================================
    // 6. ACCELERATION FEEDFORWARD (not used in AP_Follow)
    // =========================================================================
    Eigen::Vector2d accel_ff{0.0, 0.0};
    
    // =========================================================================
    // 7. SHAPE VELOCITY → ACCELERATION (AP shape_vel_accel_xy)
    // =========================================================================
    ap_control::shape_vel_accel_xy(
        vel_target_xy,
        accel_ff,
        shaped_vel_xy_,         // STATE velocity
        shaped_accel_xy_,       // STATE acceleration
        params_.accel_max_ne,
        params_.jerk_max_ne,
        dt,
        true                    // limit total accel
    );
    
    // =========================================================================
    // 8. INTEGRATE VELOCITY STATE (CRITICAL)
    // =========================================================================
    shaped_vel_xy_ += shaped_accel_xy_ * dt;
    
    // =========================================================================
    // 9. VERTICAL CONTROL (AP_Follow style)
    // =========================================================================
    double z_error = params_.follow_height - drone.alt_rel;
    
    // Use scalar sqrt_controller with VELOCITY limit
    double vel_target_z = ap_control::sqrt_controller(
        std::abs(z_error),      // Use magnitude
        params_.kp_pos,         // Position P gain
        CMD_VEL_VERTICAL_MAX,   // VELOCITY limit
        dt);
    
    // Restore sign
    if (z_error < 0.0) {
        vel_target_z = -vel_target_z;
    }
    
    // Jerk limiting - shape velocity to acceleration
    ap_control::shape_vel_accel(
        vel_target_z,
        0.0,                    // No accel feedforward
        shaped_vel_z_,
        shaped_accel_z_,
        -params_.accel_max_d,
        params_.accel_max_d,
        params_.jerk_max_d,
        dt,
        true
    );
    
    // Integrate acceleration to get velocity
    shaped_vel_z_ += shaped_accel_z_ * dt;
    
    // =========================================================================
    // 10. TRANSFORM TO BODY FRAME
    // =========================================================================
    double cp = std::cos(drone.yaw);
    double sp = std::sin(drone.yaw);
    
    cmd.vel_forward =  shaped_vel_xy_.x() * cp + shaped_vel_xy_.y() * sp;
    cmd.vel_left    = -shaped_vel_xy_.x() * sp + shaped_vel_xy_.y() * cp;
    cmd.vel_up      =  shaped_vel_z_;
    cmd.yaw_rate    = 0.0;  // Will be set by YawController
    
    return cmd;
}

void FollowController::reset() {
    shaped_vel_xy_.setZero();
    shaped_accel_xy_.setZero();
    shaped_vel_z_ = 0.0;
    shaped_accel_z_ = 0.0;
}

double FollowController::calculate_follow_distance(double target_speed) const {
    if (!params_.adaptive_distance_enable) {
        return params_.follow_dist;
    }
    
    double adaptive_dist = params_.follow_dist + target_speed * params_.distance_speed_gain;
    return std::clamp(adaptive_dist, params_.follow_dist_min, params_.follow_dist_max);
}

Eigen::Vector2d FollowController::calculate_offset_enu(const TargetState& target, 
                                                       double desired_dist) const {
    Eigen::Vector2d offset = Eigen::Vector2d::Zero();
    double target_speed = target.vel_enu.head<2>().norm();
    
    switch (params_.offset_type) {
        case OffsetType::NED:
            // NED offset → ENU (swap and negate)
            offset.x() = params_.offset_ned.y();  // East = NED_East
            offset.y() = params_.offset_ned.x();  // North = NED_North
            break;
            
        case OffsetType::RELATIVE:
            if (target_speed > SPEED_THRESHOLD_OFFSET) {
                Eigen::Vector2d offset_body;
                offset_body.x() = params_.offset_ned.y();
                offset_body.y() = params_.offset_ned.x();
                offset = rotate_vector_2d(offset_body, target.heading_rad);
            } else {
                offset.x() = params_.offset_ned.y();
                offset.y() = params_.offset_ned.x();
            }
            break;
            
        case OffsetType::VELOCITY:
        default:
            if (target_speed > SPEED_THRESHOLD_OFFSET) {
                // Position behind target along velocity vector
                offset = -target.vel_enu.head<2>() / target_speed * desired_dist;
            }
            break;
    }
    
    return offset;
}

Eigen::Vector2d FollowController::rotate_vector_2d(const Eigen::Vector2d& vec, 
                                                   double angle_rad) const {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    Eigen::Vector2d rotated;
    rotated.x() = vec.x() * c - vec.y() * s;
    rotated.y() = vec.x() * s + vec.y() * c;
    return rotated;
}

} // namespace core
} // namespace drone_follow
