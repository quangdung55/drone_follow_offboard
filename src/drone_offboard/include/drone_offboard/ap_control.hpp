/**
 * AP_CONTROL - ArduPilot Control Algorithms Ported to ROS2/Eigen
 * 
 * Ported from ArduPilot's control.cpp by Leonard Hall (2020)
 * Original: https://github.com/ArduPilot/ardupilot
 * 
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Features:
 * - Jerk-limited acceleration shaping
 * - Square-root controller for position/velocity
 * - Position-velocity-acceleration coordinated shaping
 * - Kinematic limit calculations
 * - Corner acceleration limiting with braking priority
 */

#ifndef AP_CONTROL_HPP_
#define AP_CONTROL_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace ap_control {

// ============================================================================
// CONSTANTS
// ============================================================================

constexpr double GRAVITY_MSS = 9.80665;  // m/s²
constexpr double CORNER_ACCELERATION_RATIO = 1.0 / 1.41421356237;  // 1/sqrt(2)

// Type alias for high-precision position (matches ArduPilot's postype_t)
using postype_t = double;
using Vector2p = Eigen::Vector2d;  // High-precision 2D position

// ============================================================================
// HELPER FUNCTIONS (inline for performance)
// ============================================================================

/** @brief Check if value is positive (> small epsilon) */
inline bool is_positive(double val) {
    return val > 1e-9;
}

/** @brief Check if value is negative (< -small epsilon) */
inline bool is_negative(double val) {
    return val < -1e-9;
}

/** @brief Check if value is approximately zero */
inline bool is_zero(double val) {
    return std::abs(val) < 1e-9;
}

/** @brief Safe square root that returns 0 for negative inputs */
inline double safe_sqrt(double val) {
    return (val > 0.0) ? std::sqrt(val) : 0.0;
}

/** @brief Square of a value */
inline double sq(double val) {
    return val * val;
}

/** @brief Constrain value between min and max */
inline double constrain_float(double val, double min_val, double max_val) {
    return std::clamp(val, min_val, max_val);
}

/** @brief Wrap angle to [-PI, PI] */
inline double wrap_PI(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/** @brief Convert radians to degrees */
inline double degrees(double rad) {
    return rad * 180.0 / M_PI;
}

/** @brief Convert degrees to radians */
inline double radians(double deg) {
    return deg * M_PI / 180.0;
}

// ============================================================================
// VELOCITY/ACCELERATION UPDATE FUNCTIONS
// ============================================================================

/**
 * @brief Projects velocity forward in time using acceleration, constrained by directional limit.
 * 
 * @param vel       Current velocity (modified in-place)
 * @param accel     Acceleration to apply
 * @param dt        Time step (seconds)
 * @param limit     Direction in which acceleration is constrained (0 = no limit)
 * @param vel_error Direction of velocity error (sign matters, not magnitude)
 */
void update_vel_accel(double& vel, double accel, double dt, double limit, double vel_error);

/**
 * @brief Projects position and velocity forward using acceleration with directional limits.
 * 
 * @param pos       Current position (modified in-place)
 * @param vel       Current velocity (modified in-place)
 * @param accel     Acceleration to apply
 * @param dt        Time step (seconds)
 * @param limit     Direction constraint
 * @param pos_error Position error direction
 * @param vel_error Velocity error direction
 */
void update_pos_vel_accel(postype_t& pos, double& vel, double accel, double dt, 
                          double limit, double pos_error, double vel_error);

/**
 * @brief Projects 2D velocity forward using acceleration with directional limits.
 */
void update_vel_accel_xy(Eigen::Vector2d& vel, const Eigen::Vector2d& accel, double dt, 
                         const Eigen::Vector2d& limit, const Eigen::Vector2d& vel_error);

/**
 * @brief Projects 2D position and velocity forward using acceleration with directional limits.
 */
void update_pos_vel_accel_xy(Vector2p& pos, Eigen::Vector2d& vel, const Eigen::Vector2d& accel, 
                             double dt, const Eigen::Vector2d& limit, 
                             const Eigen::Vector2d& pos_error, const Eigen::Vector2d& vel_error);

/**
 * @brief Projects 3D velocity forward using acceleration with directional limits.
 */
void update_vel_accel_xyz(Eigen::Vector3d& vel, const Eigen::Vector3d& accel, double dt, 
                          const Eigen::Vector3d& limit, const Eigen::Vector3d& vel_error);

/**
 * @brief Projects 3D position and velocity forward using acceleration with directional limits.
 */
void update_pos_vel_accel_xyz(Eigen::Vector3d& pos, Eigen::Vector3d& vel, const Eigen::Vector3d& accel, 
                              double dt, const Eigen::Vector3d& limit, 
                              const Eigen::Vector3d& pos_error, const Eigen::Vector3d& vel_error);

// ============================================================================
// ACCELERATION SHAPING FUNCTIONS
// ============================================================================

/**
 * @brief Applies jerk-limited shaping to acceleration value.
 * 
 * Constrains the rate of change of acceleration to be within ±jerk_max.
 * 
 * @param accel_desired Target acceleration
 * @param accel         Current acceleration (modified in-place)
 * @param jerk_max      Maximum jerk (m/s³)
 * @param dt            Time step (seconds)
 */
void shape_accel(double accel_desired, double& accel, double jerk_max, double dt);

/**
 * @brief Applies jerk-limited shaping to 2D acceleration vector.
 */
void shape_accel_xy(const Eigen::Vector2d& accel_desired, Eigen::Vector2d& accel,
                    double jerk_max, double dt);

/**
 * @brief Applies jerk-limited shaping to 3D acceleration (XY components only).
 */
void shape_accel_xy(const Eigen::Vector3d& accel_desired, Eigen::Vector3d& accel,
                    double jerk_max, double dt);

// ============================================================================
// VELOCITY SHAPING FUNCTIONS
// ============================================================================

/**
 * @brief Shapes velocity and acceleration using jerk-limited control.
 * 
 * Uses a square-root controller with max acceleration and jerk constraints.
 * 
 * @param vel_desired       Target velocity
 * @param accel_desired     Feedforward acceleration
 * @param vel               Current velocity
 * @param accel             Current acceleration (modified in-place)
 * @param accel_min         Minimum acceleration (negative for deceleration)
 * @param accel_max         Maximum acceleration
 * @param jerk_max          Maximum jerk
 * @param dt                Time step
 * @param limit_total_accel If true, constrains total acceleration to limits
 */
void shape_vel_accel(double vel_desired, double accel_desired,
                     double vel, double& accel,
                     double accel_min, double accel_max,
                     double jerk_max, double dt, bool limit_total_accel);

/**
 * @brief Shapes 2D velocity and acceleration using jerk-limited control.
 */
void shape_vel_accel_xy(const Eigen::Vector2d& vel_desired, const Eigen::Vector2d& accel_desired,
                        const Eigen::Vector2d& vel, Eigen::Vector2d& accel,
                        double accel_max, double jerk_max, double dt, bool limit_total_accel);

// ============================================================================
// POSITION-VELOCITY-ACCELERATION SHAPING (MAIN CONTROLLERS)
// ============================================================================

/**
 * @brief Shapes position, velocity, and acceleration using jerk-limited sqrt controller.
 * 
 * This is the primary 1D controller for smooth position tracking with:
 * - Square-root position control (prevents overshoot)
 * - Velocity feedforward support
 * - Jerk-limited acceleration output
 * 
 * @param pos_desired   Target position
 * @param vel_desired   Feedforward velocity
 * @param accel_desired Feedforward acceleration
 * @param pos           Current position
 * @param vel           Current velocity
 * @param accel         Current acceleration (modified in-place)
 * @param vel_min       Minimum velocity (negative)
 * @param vel_max       Maximum velocity
 * @param accel_min     Minimum acceleration (negative)
 * @param accel_max     Maximum acceleration
 * @param jerk_max      Maximum jerk
 * @param dt            Time step
 * @param limit_total   If true, constrains velocity and acceleration totals
 */
void shape_pos_vel_accel(postype_t pos_desired, double vel_desired, double accel_desired,
                         postype_t pos, double vel, double& accel,
                         double vel_min, double vel_max,
                         double accel_min, double accel_max,
                         double jerk_max, double dt, bool limit_total);

/**
 * @brief Shapes 2D position, velocity, and acceleration using jerk-limited sqrt controller.
 * 
 * This is the primary 2D controller for smooth lateral position tracking.
 */
void shape_pos_vel_accel_xy(const Vector2p& pos_desired, const Eigen::Vector2d& vel_desired, 
                            const Eigen::Vector2d& accel_desired,
                            const Vector2p& pos, const Eigen::Vector2d& vel, Eigen::Vector2d& accel,
                            double vel_max, double accel_max,
                            double jerk_max, double dt, bool limit_total);

/**
 * @brief Shapes angular position, velocity, and acceleration (handles angle wrapping).
 * 
 * @param angle_desired     Target angle (radians)
 * @param angle_vel_desired Feedforward angular velocity
 * @param angle_accel_desired Feedforward angular acceleration
 * @param angle             Current angle
 * @param angle_vel         Current angular velocity
 * @param angle_accel       Current angular acceleration (modified in-place)
 * @param angle_vel_max     Maximum angular velocity (rad/s)
 * @param angle_accel_max   Maximum angular acceleration (rad/s²)
 * @param angle_jerk_max    Maximum angular jerk (rad/s³)
 * @param dt                Time step
 * @param limit_total       If true, constrains totals
 */
void shape_angle_vel_accel(double angle_desired, double angle_vel_desired, double angle_accel_desired,
                           double angle, double angle_vel, double& angle_accel,
                           double angle_vel_max, double angle_accel_max,
                           double angle_jerk_max, double dt, bool limit_total);

/**
 * @brief Shapes 3D position, velocity, and acceleration with separate XY/Z parameters.
 * 
 * Combines horizontal (XY) and vertical (Z) shaping with different kinematic limits.
 * This matches ArduPilot's approach of separate NE and D axis limits.
 * 
 * @param pos_desired       Target position (3D)
 * @param vel_desired       Feedforward velocity (3D)
 * @param accel_desired     Feedforward acceleration (3D)
 * @param pos               Current position (3D)
 * @param vel               Current velocity (3D)
 * @param accel             Current acceleration (3D, modified in-place)
 * @param vel_max_xy        Max horizontal velocity
 * @param accel_max_xy      Max horizontal acceleration
 * @param jerk_max_xy       Max horizontal jerk
 * @param vel_max_z         Max vertical velocity (positive = both up and down)
 * @param accel_max_z       Max vertical acceleration
 * @param jerk_max_z        Max vertical jerk
 * @param dt                Time step
 * @param limit_total       If true, constrains totals
 */
void shape_pos_vel_accel_xyz(const Eigen::Vector3d& pos_desired, const Eigen::Vector3d& vel_desired, 
                             const Eigen::Vector3d& accel_desired,
                             const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, Eigen::Vector3d& accel,
                             double vel_max_xy, double accel_max_xy, double jerk_max_xy,
                             double vel_max_z, double accel_max_z, double jerk_max_z,
                             double dt, bool limit_total);

// ============================================================================
// SQUARE-ROOT CONTROLLER
// ============================================================================

/**
 * @brief Piecewise square-root + linear controller that limits second-order response.
 * 
 * Behaves like a P controller near the setpoint, switches to sqrt(2·a·Δx) 
 * shaping beyond a threshold to limit acceleration.
 * 
 * @param error         Position/velocity error
 * @param p             Proportional gain (1/s for position, dimensionless for velocity)
 * @param second_ord_lim Maximum acceleration allowed (m/s² or equivalent)
 * @param dt            Time step for overshoot clamping
 * @return Correction rate (velocity for position error, acceleration for velocity error)
 */
double sqrt_controller(double error, double p, double second_ord_lim, double dt);

/**
 * @brief Vector form of sqrt_controller, applied along error direction.
 */
Eigen::Vector2d sqrt_controller(const Eigen::Vector2d& error, double p, double second_ord_lim, double dt);

/**
 * @brief Inverts sqrt_controller to recover input error from output.
 * 
 * Useful for calculating required error to produce a desired rate.
 */
double inv_sqrt_controller(double output, double p, double D_max);

/**
 * @brief Computes rate-of-change implied by sqrt_controller.
 * 
 * Uses chain rule to estimate d(rate_cmd)/dt based on actual closing rate.
 */
double sqrt_controller_accel(double error, double rate_cmd, double rate_state, double p, double second_ord_lim);

/**
 * @brief Calculates stopping distance to reduce velocity to zero.
 */
double stopping_distance(double velocity, double p, double accel_max);

// ============================================================================
// ACCELERATION LIMITING
// ============================================================================

/**
 * @brief Limits 2D acceleration prioritizing cross-track over along-track.
 * 
 * @param vel       Current velocity (defines along-track direction)
 * @param accel     Acceleration vector (modified in-place)
 * @param accel_max Maximum acceleration magnitude
 * @return true if acceleration was limited
 */
bool limit_accel_xy(const Eigen::Vector2d& vel, Eigen::Vector2d& accel, double accel_max);

/**
 * @brief Limits 2D acceleration with direction-dependent prioritization.
 * 
 * When braking (negative along-track), braking is prioritized.
 * When accelerating, cross-track is prioritized.
 * 
 * @param vel       Current velocity
 * @param accel     Acceleration vector (modified in-place)
 * @param accel_max Maximum acceleration magnitude
 * @return true if limiting was applied
 */
bool limit_accel_corner_xy(const Eigen::Vector2d& vel, Eigen::Vector2d& accel, double accel_max);

// ============================================================================
// KINEMATIC LIMIT CALCULATIONS
// ============================================================================

/**
 * @brief Computes maximum acceleration/velocity in a 3D direction with axis limits.
 * 
 * @param direction   Desired direction of travel (non-zero)
 * @param max_xy      Maximum horizontal limit
 * @param max_z_neg   Maximum downward limit
 * @param max_z_pos   Maximum upward limit
 * @return Maximum achievable magnitude in that direction
 */
double kinematic_limit(const Eigen::Vector3d& direction, double max_xy, double max_z_neg, double max_z_pos);

/**
 * @brief Computes maximum magnitude for separate XY and Z components.
 */
double kinematic_limit(double segment_length_xy, double segment_length_z, 
                       double max_xy, double max_z_neg, double max_z_pos);

// ============================================================================
// INPUT SHAPING & CONVERSION UTILITIES
// ============================================================================

/**
 * @brief Applies exponential curve to normalized input [-1, 1].
 * 
 * Typically used for pilot stick input response shaping.
 * 
 * @param input Normalized input (-1 to 1)
 * @param expo  Expo factor (0 = linear, closer to 1 = more curvature)
 * @return Shaped output
 */
double input_expo(double input, double expo);

/**
 * @brief Converts lean angle (radians) to horizontal acceleration.
 */
double angle_rad_to_accel_mss(double angle_rad);

/**
 * @brief Converts lean angle (degrees) to horizontal acceleration.
 */
double angle_deg_to_accel_mss(double angle_deg);

/**
 * @brief Converts horizontal acceleration to lean angle (radians).
 */
double accel_mss_to_angle_rad(double accel_mss);

/**
 * @brief Converts horizontal acceleration to lean angle (degrees).
 */
double accel_mss_to_angle_deg(double accel_mss);

/**
 * @brief Converts normalized roll/pitch input to target angles.
 * 
 * @param roll_in_norm    Normalized roll input (-1 to 1)
 * @param pitch_in_norm   Normalized pitch input (-1 to 1)
 * @param angle_max_rad   Maximum lean angle
 * @param angle_limit_rad Secondary limit
 * @param roll_out_rad    Output roll angle (radians)
 * @param pitch_out_rad   Output pitch angle (radians)
 */
void rc_input_to_roll_pitch_rad(double roll_in_norm, double pitch_in_norm, 
                                double angle_max_rad, double angle_limit_rad, 
                                double& roll_out_rad, double& pitch_out_rad);

}  // namespace ap_control

#endif  // AP_CONTROL_HPP_
