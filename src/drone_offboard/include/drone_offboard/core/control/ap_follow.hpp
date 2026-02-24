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
 * 
 * Features:
 * - Kinematic input shaping for smooth target estimation
 * - Position/velocity/acceleration estimation with jerk limiting
 * - Heading estimation with angular shaping
 * - Offset handling (NED and body-relative frames)
 * - Estimate validity checking
 */

#ifndef AP_FOLLOW_HPP_
#define AP_FOLLOW_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include "ap_control.hpp"

namespace ap_follow {

// ============================================================================
// CONSTANTS
// ============================================================================

constexpr double AP_FOLLOW_TIMEOUT_SEC = 3.0;           // Position estimate timeout
constexpr double AP_FOLLOW_POS_P_DEFAULT = 0.1;         // Position error P gain

// Offset types
enum class OffsetType : int {
    NED = 0,        // Offsets are in North-East-Down frame
    RELATIVE = 1    // Offsets are relative to lead vehicle heading (FRD)
};

// Yaw behavior options
enum class YawBehavior : int {
    NONE = 0,              // No yaw control
    FACE_LEAD = 1,         // Face the lead vehicle
    SAME_AS_LEAD = 2,      // Match lead vehicle's heading
    DIRECTION_OF_FLIGHT = 3 // Point in direction of flight
};

// ============================================================================
// TARGET STATE STRUCTURE
// ============================================================================

/**
 * @brief Raw target state from sensor/MAVLink
 */
struct TargetMeasurement {
    Eigen::Vector3d pos_ned_m = Eigen::Vector3d::Zero();      // Position in NED frame (m)
    Eigen::Vector3d vel_ned_ms = Eigen::Vector3d::Zero();     // Velocity in NED frame (m/s)
    Eigen::Vector3d accel_ned_mss = Eigen::Vector3d::Zero();  // Acceleration in NED frame (m/s²)
    double heading_rad = 0.0;                                  // Heading (rad)
    double heading_rate_rads = 0.0;                            // Heading rate (rad/s)
    double timestamp_sec = 0.0;                                // Timestamp of measurement
    bool valid = false;                                        // Measurement validity
};

/**
 * @brief Shaped/estimated target state after kinematic filtering
 */
struct TargetEstimate {
    Eigen::Vector3d pos_ned_m = Eigen::Vector3d::Zero();      // Estimated position (m)
    Eigen::Vector3d vel_ned_ms = Eigen::Vector3d::Zero();     // Estimated velocity (m/s)
    Eigen::Vector3d accel_ned_mss = Eigen::Vector3d::Zero();  // Estimated acceleration (m/s²)
    double heading_rad = 0.0;                                  // Estimated heading (rad)
    double heading_rate_rads = 0.0;                            // Estimated heading rate (rad/s)
    double heading_accel_radss = 0.0;                          // Estimated heading acceleration (rad/s²)
    bool valid = false;                                        // Estimate validity
};

/**
 * @brief Kinematic shaping parameters (per-axis)
 */
struct KinematicParams {
    double accel_max_ne_mss = 2.5;    // Max horizontal acceleration (m/s²)
    double jerk_max_ne_msss = 5.0;    // Max horizontal jerk (m/s³)
    double accel_max_d_mss = 2.5;     // Max vertical acceleration (m/s²)
    double jerk_max_d_msss = 5.0;     // Max vertical jerk (m/s³)
    double accel_max_h_radss = 1.57;  // Max heading acceleration (rad/s²) ~90 deg/s²
    double jerk_max_h_radsss = 6.28;  // Max heading jerk (rad/s³) ~360 deg/s³
    double timeout_sec = 3.0;         // Estimate timeout (s)
    double dist_max_m = 100.0;        // Max follow distance (m), 0 = disabled
    double pos_p = 0.1;               // Position P gain
    
    bool is_valid() const {
        return (accel_max_ne_mss > 0.0) && (jerk_max_ne_msss > 0.0) &&
               (accel_max_d_mss > 0.0) && (jerk_max_d_msss > 0.0) &&
               (accel_max_h_radss > 0.0) && (jerk_max_h_radsss > 0.0);
    }
};

/**
 * @brief Follow offset configuration
 */
struct FollowOffset {
    Eigen::Vector3d offset_m = Eigen::Vector3d::Zero();  // Offset vector (m)
    OffsetType type = OffsetType::NED;                   // Offset frame type
    
    bool is_zero() const {
        return offset_m.squaredNorm() < 1e-9;
    }
};

// ============================================================================
// AP_FOLLOW ESTIMATOR CLASS
// ============================================================================

/**
 * @brief Target following estimator with kinematic input shaping
 * 
 * This class implements ArduPilot's AP_Follow estimation logic:
 * - Receives raw target measurements (position, velocity, heading)
 * - Applies kinematic shaping to smooth the estimate
 * - Handles offsets in NED or body-relative frames
 * - Validates estimates and handles timeouts
 */
class FollowEstimator {
public:
    FollowEstimator() = default;
    ~FollowEstimator() = default;

    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set kinematic shaping parameters
     */
    void set_kinematic_params(const KinematicParams& params) {
        params_ = params;
    }
    
    /**
     * @brief Get current kinematic parameters
     */
    const KinematicParams& get_kinematic_params() const {
        return params_;
    }
    
    /**
     * @brief Set follow offset
     */
    void set_offset(const FollowOffset& offset) {
        offset_ = offset;
    }
    
    /**
     * @brief Get current offset
     */
    const FollowOffset& get_offset() const {
        return offset_;
    }

    // ========================================================================
    // Target Update
    // ========================================================================
    
    /**
     * @brief Update target measurement from sensor/MAVLink
     * 
     * @param measurement New target measurement
     */
    void update_target(const TargetMeasurement& measurement);
    
    /**
     * @brief Update the kinematic estimate (call this at control loop rate)
     * 
     * @param current_time_sec Current time in seconds
     */
    void update_estimate(double current_time_sec);

    // ========================================================================
    // Estimate Retrieval
    // ========================================================================
    
    /**
     * @brief Get raw (unfiltered) target state
     */
    const TargetMeasurement& get_target() const { return target_; }
    
    /**
     * @brief Get filtered target estimate (no offset)
     */
    const TargetEstimate& get_estimate() const { return estimate_; }
    
    /**
     * @brief Get filtered target estimate with offset applied
     */
    bool get_estimate_with_offset(TargetEstimate& ofs_estimate) const;
    
    /**
     * @brief Get position, velocity, acceleration with offset (convenience)
     */
    bool get_ofs_pos_vel_accel_ned(Eigen::Vector3d& pos, Eigen::Vector3d& vel, 
                                    Eigen::Vector3d& accel) const;
    
    /**
     * @brief Get heading and heading rate
     */
    bool get_heading_rate(double& heading_rad, double& heading_rate_rads) const;
    
    /**
     * @brief Check if estimate is valid
     */
    bool is_valid() const { return estimate_.valid; }
    
    /**
     * @brief Check if target has been received recently
     */
    bool have_target() const;

    // ========================================================================
    // Distance/Bearing
    // ========================================================================
    
    /**
     * @brief Calculate distance to target with offset
     * 
     * @param vehicle_pos_ned Current vehicle position in NED
     * @return Distance in meters (negative if invalid)
     */
    double get_distance_to_target(const Eigen::Vector3d& vehicle_pos_ned) const;
    
    /**
     * @brief Calculate bearing to target with offset
     * 
     * @param vehicle_pos_ned Current vehicle position in NED
     * @return Bearing in radians (0 = North, positive = clockwise)
     */
    double get_bearing_to_target(const Eigen::Vector3d& vehicle_pos_ned) const;

    // ========================================================================
    // Error Checking
    // ========================================================================
    
    /**
     * @brief Check if estimate error is too large (should reset)
     */
    bool estimate_error_too_large() const;
    
    /**
     * @brief Reset the estimator state
     */
    void reset();

    // ========================================================================
    // Offset Helpers
    // ========================================================================
    
    /**
     * @brief Initialize offset from current relative position
     * 
     * @param vehicle_pos_ned Current vehicle position in NED
     * @param use_relative If true, use body-relative frame
     */
    void init_offset_from_position(const Eigen::Vector3d& vehicle_pos_ned, bool use_relative);
    
    /**
     * @brief Clear offset to zero
     */
    void clear_offset() {
        offset_.offset_m.setZero();
    }

private:
    // ========================================================================
    // Internal Helpers
    // ========================================================================
    
    /**
     * @brief Calculate max velocity change under jerk-limited profile
     */
    double calc_max_velocity_change(double accel_max, double jerk_max, double timeout_sec) const;
    
    /**
     * @brief Apply offset to estimate
     */
    void apply_offset(const TargetEstimate& estimate, TargetEstimate& ofs_estimate) const;
    
    /**
     * @brief Rotate vector by heading angle
     */
    static Eigen::Vector3d rotate_by_heading(const Eigen::Vector3d& vec, double heading_rad);

    // ========================================================================
    // State
    // ========================================================================
    
    KinematicParams params_;
    FollowOffset offset_;
    
    TargetMeasurement target_;       // Raw target measurement
    TargetEstimate estimate_;        // Shaped estimate (no offset)
    TargetEstimate ofs_estimate_;    // Shaped estimate with offset
    
    double last_target_update_sec_ = 0.0;
    double last_estimate_update_sec_ = 0.0;
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Calculate stopping distance for a given velocity
 */
inline double calc_stopping_distance(double velocity, double pos_p, double accel_max) {
    return ap_control::stopping_distance(velocity, pos_p, accel_max);
}

/**
 * @brief Calculate max velocity for a given distance
 */
inline double calc_max_velocity_for_distance(double distance, double pos_p, double accel_max) {
    return ap_control::sqrt_controller(distance, pos_p, accel_max, 0.0);
}

}  // namespace ap_follow

#endif  // AP_FOLLOW_HPP_
