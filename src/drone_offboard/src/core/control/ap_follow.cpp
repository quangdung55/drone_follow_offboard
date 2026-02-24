/**
 * AP_FOLLOW - ArduPilot Follow Mode Estimation Library
 * 
 * Ported from ArduPilot's AP_Follow.cpp
 * Original: https://github.com/ArduPilot/ardupilot
 * 
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "drone_offboard/core/control/ap_follow.hpp"
#include <iostream>

namespace ap_follow {

// ============================================================================
// TARGET UPDATE
// ============================================================================

void FollowEstimator::update_target(const TargetMeasurement& measurement)
{
    if (!measurement.valid) {
        return;
    }
    
    // Store the new measurement
    target_ = measurement;
    last_target_update_sec_ = measurement.timestamp_sec;
}

// ============================================================================
// ESTIMATE UPDATE (Main kinematic shaping loop)
// ============================================================================

void FollowEstimator::update_estimate(double current_time_sec)
{
    // Check for target validity
    if (!have_target()) {
        estimate_.valid = false;
        ofs_estimate_.valid = false;
        return;
    }
    
    // Calculate time since last location update
    const double dt_target = current_time_sec - last_target_update_sec_;
    
    // Project target's position and velocity forward using simple kinematics
    // P = P0 + V*dt + 0.5*A*dt²
    Eigen::Vector3d delta_pos_m = target_.vel_ned_ms * dt_target + 
                                   target_.accel_ned_mss * 0.5 * dt_target * dt_target;
    Eigen::Vector3d delta_vel_ms = target_.accel_ned_mss * dt_target;
    double delta_heading_rad = target_.heading_rate_rads * dt_target;
    
    // Calculate time since last estimation update
    const double e_dt = current_time_sec - last_estimate_update_sec_;
    
    if (estimate_.valid && params_.is_valid() && e_dt > 0.001 && e_dt < 0.5) {
        // ====================================================================
        // KINEMATIC INPUT SHAPING (ArduPilot style)
        // ====================================================================
        
        // --- Horizontal (XY) Position/Velocity/Acceleration Shaping ---
        
        // First: update estimate forward in time
        Eigen::Vector2d est_pos_xy = estimate_.pos_ned_m.head<2>();
        Eigen::Vector2d est_vel_xy = estimate_.vel_ned_ms.head<2>();
        Eigen::Vector2d est_accel_xy = estimate_.accel_ned_mss.head<2>();
        
        // Project estimate forward
        ap_control::update_pos_vel_accel_xy(
            est_pos_xy, est_vel_xy, est_accel_xy, e_dt,
            Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()
        );
        
        // Target position/velocity/acceleration (projected)
        Eigen::Vector2d target_pos_xy = (target_.pos_ned_m + delta_pos_m).head<2>();
        Eigen::Vector2d target_vel_xy = (target_.vel_ned_ms + delta_vel_ms).head<2>();
        Eigen::Vector2d target_accel_xy = target_.accel_ned_mss.head<2>();
        
        // Apply horizontal shaping to refine estimate toward projected target
        ap_control::shape_pos_vel_accel_xy(
            target_pos_xy,              // desired position
            target_vel_xy,              // desired velocity
            target_accel_xy,            // desired acceleration
            est_pos_xy,                 // current position
            est_vel_xy,                 // current velocity
            est_accel_xy,               // current acceleration (output)
            0.0,                        // vel_max (0 = no limit)
            params_.accel_max_ne_mss,   // accel_max
            params_.jerk_max_ne_msss,   // jerk_max
            e_dt,                       // dt
            false                       // limit_total
        );
        
        // Write back XY components
        estimate_.pos_ned_m.head<2>() = est_pos_xy;
        estimate_.vel_ned_ms.head<2>() = est_vel_xy;
        estimate_.accel_ned_mss.head<2>() = est_accel_xy;
        
        // --- Vertical (Z) Position/Velocity/Acceleration Shaping ---
        
        double est_pos_z = estimate_.pos_ned_m.z();
        double est_vel_z = estimate_.vel_ned_ms.z();
        double est_accel_z = estimate_.accel_ned_mss.z();
        
        // Project estimate forward
        ap_control::update_pos_vel_accel(est_pos_z, est_vel_z, est_accel_z, e_dt, 0.0, 0.0, 0.0);
        
        // Target Z (projected)
        double target_pos_z = target_.pos_ned_m.z() + delta_pos_m.z();
        double target_vel_z = target_.vel_ned_ms.z() + delta_vel_ms.z();
        double target_accel_z = target_.accel_ned_mss.z();
        
        // Apply vertical shaping
        ap_control::shape_pos_vel_accel(
            target_pos_z,               // desired position
            target_vel_z,               // desired velocity  
            target_accel_z,             // desired acceleration
            est_pos_z,                  // current position
            est_vel_z,                  // current velocity
            est_accel_z,                // current acceleration (output)
            -100.0,                     // vel_min (large negative)
            100.0,                      // vel_max (large positive)
            -params_.accel_max_d_mss,   // accel_min
            params_.accel_max_d_mss,    // accel_max
            params_.jerk_max_d_msss,    // jerk_max
            e_dt,                       // dt
            false                       // limit_total
        );
        
        // Write back Z component
        estimate_.pos_ned_m.z() = est_pos_z;
        estimate_.vel_ned_ms.z() = est_vel_z;
        estimate_.accel_ned_mss.z() = est_accel_z;
        
        // --- Heading (Angular) Shaping ---
        
        double target_heading = target_.heading_rad + delta_heading_rad;
        double target_heading_rate = target_.heading_rate_rads;
        
        ap_control::shape_angle_vel_accel(
            target_heading,                 // desired angle
            target_heading_rate,            // desired angular velocity
            0.0,                            // desired angular acceleration
            estimate_.heading_rad,          // current angle
            estimate_.heading_rate_rads,    // current angular velocity
            estimate_.heading_accel_radss,  // current angular acceleration (output)
            100.0,                          // angle_vel_max (large)
            params_.accel_max_h_radss,      // angle_accel_max
            params_.jerk_max_h_radsss,      // angle_jerk_max
            e_dt,                           // dt
            false                           // limit_total
        );
        
        // Update heading with proper wrapping
        ap_control::postype_t heading_pos = estimate_.heading_rad;
        ap_control::update_pos_vel_accel(
            heading_pos, 
            estimate_.heading_rate_rads, 
            estimate_.heading_accel_radss, 
            e_dt, 0.0, 0.0, 0.0
        );
        estimate_.heading_rad = ap_control::wrap_PI(heading_pos);
        
    } else {
        // ====================================================================
        // NO VALID ESTIMATE: Initialize from target
        // ====================================================================
        
        estimate_.pos_ned_m = target_.pos_ned_m + delta_pos_m;
        estimate_.vel_ned_ms = target_.vel_ned_ms + delta_vel_ms;
        estimate_.accel_ned_mss = target_.accel_ned_mss;
        estimate_.heading_rad = ap_control::wrap_PI(target_.heading_rad + delta_heading_rad);
        estimate_.heading_rate_rads = target_.heading_rate_rads;
        estimate_.heading_accel_radss = 0.0;
        estimate_.valid = true;
    }
    
    // Apply offset to get final estimate
    apply_offset(estimate_, ofs_estimate_);
    
    last_estimate_update_sec_ = current_time_sec;
    
    // Check if estimate error is too large
    if (estimate_error_too_large()) {
        std::cerr << "[ap_follow] Estimate error too large, resetting..." << std::endl;
        estimate_.valid = false;
        ofs_estimate_.valid = false;
    }
}

// ============================================================================
// OFFSET APPLICATION
// ============================================================================

void FollowEstimator::apply_offset(const TargetEstimate& estimate, TargetEstimate& ofs_estimate) const
{
    if (!estimate.valid) {
        ofs_estimate.valid = false;
        return;
    }
    
    Eigen::Vector3d offset_m = offset_.offset_m;
    
    if (offset_.is_zero() || offset_.type == OffsetType::NED) {
        // Offsets are in NED frame: simple addition
        ofs_estimate.pos_ned_m = estimate.pos_ned_m + offset_m;
        ofs_estimate.vel_ned_ms = estimate.vel_ned_ms;
        ofs_estimate.accel_ned_mss = estimate.accel_ned_mss;
    } else {
        // Offsets are in body (FRD) frame: rotate by heading
        Eigen::Vector3d rotated_offset = rotate_by_heading(offset_m, estimate.heading_rad);
        ofs_estimate.pos_ned_m = estimate.pos_ned_m + rotated_offset;
        ofs_estimate.vel_ned_ms = estimate.vel_ned_ms;
        ofs_estimate.accel_ned_mss = estimate.accel_ned_mss;
        
        // With kinematic shaping, we can add velocity/acceleration from rotation
        if (params_.is_valid()) {
            // Cross product: offset × [0, 0, 1] = [-offset.y, offset.x, 0]
            // This represents the tangent direction for rotational motion
            Eigen::Vector3d offset_cross(-rotated_offset.y(), rotated_offset.x(), 0.0);
            
            // Add rotational velocity: v_offset = omega × r
            ofs_estimate.vel_ned_ms += offset_cross * estimate.heading_rate_rads;
            
            // Add rotational acceleration: a_offset = alpha × r
            ofs_estimate.accel_ned_mss += offset_cross * estimate.heading_accel_radss;
        }
    }
    
    ofs_estimate.heading_rad = estimate.heading_rad;
    ofs_estimate.heading_rate_rads = estimate.heading_rate_rads;
    ofs_estimate.heading_accel_radss = estimate.heading_accel_radss;
    ofs_estimate.valid = true;
}

Eigen::Vector3d FollowEstimator::rotate_by_heading(const Eigen::Vector3d& vec, double heading_rad)
{
    double cos_h = std::cos(heading_rad);
    double sin_h = std::sin(heading_rad);
    
    Eigen::Vector3d rotated;
    rotated.x() = vec.x() * cos_h - vec.y() * sin_h;  // North
    rotated.y() = vec.x() * sin_h + vec.y() * cos_h;  // East
    rotated.z() = vec.z();                             // Down (unchanged)
    
    return rotated;
}

// ============================================================================
// ESTIMATE RETRIEVAL
// ============================================================================

bool FollowEstimator::get_estimate_with_offset(TargetEstimate& ofs_est) const
{
    if (!ofs_estimate_.valid) {
        return false;
    }
    ofs_est = ofs_estimate_;
    return true;
}

bool FollowEstimator::get_ofs_pos_vel_accel_ned(Eigen::Vector3d& pos, Eigen::Vector3d& vel, 
                                                 Eigen::Vector3d& accel) const
{
    if (!ofs_estimate_.valid) {
        return false;
    }
    pos = ofs_estimate_.pos_ned_m;
    vel = ofs_estimate_.vel_ned_ms;
    accel = ofs_estimate_.accel_ned_mss;
    return true;
}

bool FollowEstimator::get_heading_rate(double& heading_rad, double& heading_rate_rads) const
{
    if (!estimate_.valid) {
        return false;
    }
    heading_rad = estimate_.heading_rad;
    heading_rate_rads = estimate_.heading_rate_rads;
    return true;
}

bool FollowEstimator::have_target() const
{
    if (!target_.valid) {
        return false;
    }
    
    // Note: Caller should provide current time and check timeout
    // This is a simplified check - full implementation needs current time
    return true;
}

// ============================================================================
// DISTANCE/BEARING
// ============================================================================

double FollowEstimator::get_distance_to_target(const Eigen::Vector3d& vehicle_pos_ned) const
{
    if (!ofs_estimate_.valid) {
        return -1.0;
    }
    
    Eigen::Vector2d dist_xy = (ofs_estimate_.pos_ned_m - vehicle_pos_ned).head<2>();
    return dist_xy.norm();
}

double FollowEstimator::get_bearing_to_target(const Eigen::Vector3d& vehicle_pos_ned) const
{
    if (!ofs_estimate_.valid) {
        return 0.0;
    }
    
    Eigen::Vector2d dist_xy = (ofs_estimate_.pos_ned_m - vehicle_pos_ned).head<2>();
    if (dist_xy.squaredNorm() < 1e-9) {
        return 0.0;
    }
    
    // atan2(East, North) gives bearing from North
    return std::atan2(dist_xy.y(), dist_xy.x());
}

// ============================================================================
// ERROR CHECKING
// ============================================================================

bool FollowEstimator::estimate_error_too_large() const
{
    if (!estimate_.valid || !target_.valid) {
        return false;
    }
    
    const double timeout_sec = params_.timeout_sec;
    
    // Calculate position thresholds based on maximum acceleration for timeout duration
    // Position change = 0.5 * a * t^2 for acceleration then deceleration
    const double pos_thresh_horiz_m = params_.accel_max_ne_mss * ap_control::sq(timeout_sec * 0.5);
    const double pos_thresh_vert_m = params_.accel_max_d_mss * ap_control::sq(timeout_sec * 0.5);
    
    // Calculate velocity thresholds using jerk-limited profile
    const double vel_thresh_horiz_ms = calc_max_velocity_change(
        params_.accel_max_ne_mss, params_.jerk_max_ne_msss, timeout_sec);
    const double vel_thresh_vert_ms = calc_max_velocity_change(
        params_.accel_max_d_mss, params_.jerk_max_d_msss, timeout_sec);
    
    // Calculate current position and velocity errors
    const Eigen::Vector3d pos_error = estimate_.pos_ned_m - target_.pos_ned_m;
    const Eigen::Vector3d vel_error = estimate_.vel_ned_ms - target_.vel_ned_ms;
    
    const double pos_error_horiz = pos_error.head<2>().norm();
    const double pos_error_vert = std::abs(pos_error.z());
    const double vel_error_horiz = vel_error.head<2>().norm();
    const double vel_error_vert = std::abs(vel_error.z());
    
    // Check if any threshold is exceeded
    const bool pos_horiz_bad = pos_error_horiz > pos_thresh_horiz_m;
    const bool vel_horiz_bad = vel_error_horiz > vel_thresh_horiz_ms;
    const bool pos_vert_bad = pos_error_vert > pos_thresh_vert_m;
    const bool vel_vert_bad = vel_error_vert > vel_thresh_vert_ms;
    
    return pos_horiz_bad || vel_horiz_bad || pos_vert_bad || vel_vert_bad;
}

double FollowEstimator::calc_max_velocity_change(double accel_max, double jerk_max, 
                                                   double timeout_sec) const
{
    // Time to ramp acceleration from 0 to accel_max
    const double t_jerk = accel_max / jerk_max;
    const double t_total_jerk = 2.0 * t_jerk;
    
    if (timeout_sec >= t_total_jerk) {
        // Enough time for trapezoidal profile: ramp up, constant, ramp down
        const double t_const = timeout_sec - t_total_jerk;
        const double delta_v_jerk = 0.5 * accel_max * t_jerk;
        const double delta_v_const = accel_max * t_const;
        return 2.0 * delta_v_jerk + delta_v_const;
    } else {
        // Timeout too short: pure triangular profile
        const double t_half = timeout_sec * 0.5;
        return 0.5 * jerk_max * t_half * t_half;
    }
}

void FollowEstimator::reset()
{
    target_.valid = false;
    estimate_.valid = false;
    ofs_estimate_.valid = false;
    last_target_update_sec_ = 0.0;
    last_estimate_update_sec_ = 0.0;
}

// ============================================================================
// OFFSET INITIALIZATION
// ============================================================================

void FollowEstimator::init_offset_from_position(const Eigen::Vector3d& vehicle_pos_ned, 
                                                  bool use_relative)
{
    if (!estimate_.valid) {
        return;
    }
    
    // Calculate distance vector from target to vehicle
    Eigen::Vector3d dist_vec = vehicle_pos_ned - estimate_.pos_ned_m;
    
    if (use_relative) {
        // Rotate into body-relative (FRD) frame
        // Inverse rotation: rotate by -heading
        offset_.offset_m = rotate_by_heading(dist_vec, -estimate_.heading_rad);
        offset_.type = OffsetType::RELATIVE;
    } else {
        // NED frame: direct assignment
        offset_.offset_m = dist_vec;
        offset_.type = OffsetType::NED;
    }
}

}  // namespace ap_follow
