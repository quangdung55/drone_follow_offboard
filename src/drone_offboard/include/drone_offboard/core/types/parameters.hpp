#ifndef DRONE_OFFBOARD_CORE_TYPES_PARAMETERS_HPP_
#define DRONE_OFFBOARD_CORE_TYPES_PARAMETERS_HPP_

/**
 * @file parameters.hpp
 * @brief Controller parameters (NO ROS DEPENDENCY)
 */

#include <Eigen/Dense>
#include "common.hpp"

namespace drone_follow {
namespace core {

/**
 * @brief Follow controller parameters
 */
struct FollowControlParams {
    // Gains
    double kp_pos = 0.1;              // Position P gain (sqrt controller)
    double kp_vel_z = 1.5;            // Vertical velocity P gain
    
    // Kinematic limits
    double accel_max_ne = 2.5;        // Horizontal acceleration limit (m/s²)
    double jerk_max_ne = 5.0;         // Horizontal jerk limit (m/s³)
    double accel_max_d = 1.0;         // Vertical acceleration limit (m/s²)
    double jerk_max_d = 2.0;          // Vertical jerk limit (m/s³)
    
    // Follow parameters
    double follow_height = 3.0;       // Desired height above target (m)
    double follow_dist = 5.0;         // Desired horizontal distance (m)
    
    // Offset
    OffsetType offset_type = OffsetType::VELOCITY;
    Eigen::Vector3d offset_ned = Eigen::Vector3d::Zero();  // (North, East, Down)
    
    // Adaptive distance
    bool adaptive_distance_enable = false;
    double follow_dist_min = 3.0;
    double follow_dist_max = 15.0;
    double distance_speed_gain = 1.5;
    
    // Terrain follow
    bool terrain_follow_enable = true;
};

/**
 * @brief Yaw controller parameters
 */
struct YawControlParams {
    // Gains
    double kp_yaw = 0.05;                  // Yaw rate P gain (gimbal)
    double heading_follow_kp = 0.5;        // Heading follow P gain
    
    // Kinematic limits
    double accel_max_h = 90.0 * DEG_TO_RAD;      // deg/s² → rad/s²
    double jerk_max_h = 360.0 * DEG_TO_RAD;      // deg/s³ → rad/s³
    
    // Deadzone and blending
    double gimbal_deadzone = 5.0;          // degrees
    bool heading_blend_enable = false;
    double heading_blend_weight = 0.3;     // 0=gimbal only, 1=heading only
};

/**
 * @brief Estimator parameters
 */
struct EstimatorParams {
    // Alpha-Beta filter
    bool filter_enable = true;
    double filter_alpha = 0.7;
    double filter_beta = 0.1;
    double filter_beta_fast = 0.1;
    double filter_beta_slow = 0.05;
    double filter_residual_threshold = 2.0;  // m
    
    // Jitter correction
    bool jitter_correction_enable = true;
    int jitter_max_lag_ms = 500;
    int jitter_convergence_loops = 100;
    
    // Timeout
    double timeout_sec = 3.0;
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_TYPES_PARAMETERS_HPP_
