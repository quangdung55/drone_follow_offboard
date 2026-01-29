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
 */

#include "drone_offboard/ap_control.hpp"
#include <iostream>

namespace ap_control {

// ============================================================================
// VELOCITY/ACCELERATION UPDATE FUNCTIONS
// ============================================================================

void update_vel_accel(double& vel, double accel, double dt, double limit, double vel_error)
{
    double delta_vel = accel * dt;
    // do not add delta_vel if it will increase the velocity error in the direction of limit
    // unless adding delta_vel will reduce vel towards zero
    if (is_positive(delta_vel * limit) && is_positive(vel_error * limit)) {
        if (is_negative(vel * limit)) {
            delta_vel = constrain_float(delta_vel, -std::abs(vel), std::abs(vel));
        } else {
            delta_vel = 0.0;
        }
    }
    vel += delta_vel;
}

void update_pos_vel_accel(postype_t& pos, double& vel, double accel, double dt, 
                          double limit, double pos_error, double vel_error)
{
    // move position and velocity forward by dt if it does not increase error when limited.
    double delta_pos = vel * dt + accel * 0.5 * sq(dt);
    // do not add delta_pos if it will increase the velocity error in the direction of limit
    if (is_positive(delta_pos * limit) && is_positive(pos_error * limit)) {
        delta_pos = 0.0;
    }
    pos += delta_pos;

    update_vel_accel(vel, accel, dt, limit, vel_error);
}

void update_vel_accel_xy(Eigen::Vector2d& vel, const Eigen::Vector2d& accel, double dt, 
                         const Eigen::Vector2d& limit, const Eigen::Vector2d& vel_error)
{
    // increase velocity by acceleration * dt if it does not increase error when limited.
    Eigen::Vector2d delta_vel = accel * dt;
    if (!is_zero(limit.squaredNorm()) && !is_zero(delta_vel.squaredNorm())) {
        // check if delta_vel will increase the velocity error in the direction of limit
        if (is_positive(delta_vel.dot(limit)) && is_positive(vel_error.dot(limit)) && !is_negative(vel.dot(limit))) {
            delta_vel.setZero();
        }
    }
    vel += delta_vel;
}

void update_pos_vel_accel_xy(Vector2p& pos, Eigen::Vector2d& vel, const Eigen::Vector2d& accel, 
                             double dt, const Eigen::Vector2d& limit, 
                             const Eigen::Vector2d& pos_error, const Eigen::Vector2d& vel_error)
{
    // move position and velocity forward by dt.
    Eigen::Vector2d delta_pos = vel * dt + accel * 0.5 * sq(dt);

    if (!is_zero(limit.squaredNorm())) {
        // zero delta_pos if it will increase the velocity error in the direction of limit
        if (is_positive(delta_pos.dot(limit)) && is_positive(pos_error.dot(limit))) {
            delta_pos.setZero();
        }
    }

    pos += delta_pos;

    update_vel_accel_xy(vel, accel, dt, limit, vel_error);
}

void update_vel_accel_xyz(Eigen::Vector3d& vel, const Eigen::Vector3d& accel, double dt, 
                          const Eigen::Vector3d& limit, const Eigen::Vector3d& vel_error)
{
    // Update XY components
    Eigen::Vector2d vel_xy = vel.head<2>();
    Eigen::Vector2d accel_xy = accel.head<2>();
    Eigen::Vector2d limit_xy = limit.head<2>();
    Eigen::Vector2d vel_error_xy = vel_error.head<2>();
    
    update_vel_accel_xy(vel_xy, accel_xy, dt, limit_xy, vel_error_xy);
    vel.head<2>() = vel_xy;
    
    // Update Z component
    double vel_z = vel.z();
    update_vel_accel(vel_z, accel.z(), dt, limit.z(), vel_error.z());
    vel.z() = vel_z;
}

void update_pos_vel_accel_xyz(Eigen::Vector3d& pos, Eigen::Vector3d& vel, const Eigen::Vector3d& accel, 
                              double dt, const Eigen::Vector3d& limit, 
                              const Eigen::Vector3d& pos_error, const Eigen::Vector3d& vel_error)
{
    // Update XY components
    Vector2p pos_xy = pos.head<2>();
    Eigen::Vector2d vel_xy = vel.head<2>();
    Eigen::Vector2d accel_xy = accel.head<2>();
    Eigen::Vector2d limit_xy = limit.head<2>();
    Eigen::Vector2d pos_error_xy = pos_error.head<2>();
    Eigen::Vector2d vel_error_xy = vel_error.head<2>();
    
    update_pos_vel_accel_xy(pos_xy, vel_xy, accel_xy, dt, limit_xy, pos_error_xy, vel_error_xy);
    pos.head<2>() = pos_xy;
    vel.head<2>() = vel_xy;
    
    // Update Z component
    postype_t pos_z = pos.z();
    double vel_z = vel.z();
    update_pos_vel_accel(pos_z, vel_z, accel.z(), dt, limit.z(), pos_error.z(), vel_error.z());
    pos.z() = pos_z;
    vel.z() = vel_z;
}

// ============================================================================
// ACCELERATION SHAPING FUNCTIONS
// ============================================================================

void shape_accel(double accel_desired, double& accel, double jerk_max, double dt)
{
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        std::cerr << "[ap_control] shape_accel: invalid jerk_max" << std::endl;
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        double accel_delta = accel_desired - accel;
        accel_delta = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
        accel += accel_delta;
    }
}

void shape_accel_xy(const Eigen::Vector2d& accel_desired, Eigen::Vector2d& accel,
                    double jerk_max, double dt)
{
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        std::cerr << "[ap_control] shape_accel_xy: invalid jerk_max" << std::endl;
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        Eigen::Vector2d accel_delta = accel_desired - accel;
        double delta_norm = accel_delta.norm();
        if (delta_norm > jerk_max * dt) {
            accel_delta *= (jerk_max * dt / delta_norm);
        }
        accel += accel_delta;
    }
}

void shape_accel_xy(const Eigen::Vector3d& accel_desired, Eigen::Vector3d& accel,
                    double jerk_max, double dt)
{
    Eigen::Vector2d accel_desired_2d(accel_desired.x(), accel_desired.y());
    Eigen::Vector2d accel_2d(accel.x(), accel.y());

    shape_accel_xy(accel_desired_2d, accel_2d, jerk_max, dt);
    accel.x() = accel_2d.x();
    accel.y() = accel_2d.y();
}

// ============================================================================
// VELOCITY SHAPING FUNCTIONS
// ============================================================================

void shape_vel_accel(double vel_desired, double accel_desired,
                     double vel, double& accel,
                     double accel_min, double accel_max,
                     double jerk_max, double dt, bool limit_total_accel)
{
    // sanity check accel_min, accel_max and jerk_max.
    if (!is_negative(accel_min) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        std::cerr << "[ap_control] shape_vel_accel: invalid limits" << std::endl;
        return;
    }

    // velocity error to be corrected
    double vel_error = vel_desired - vel;

    // Calculate time constants and limits to ensure stable operation
    double KPa;
    if (is_positive(vel_error)) {
        KPa = jerk_max / accel_max;
    } else {
        KPa = jerk_max / (-accel_min);
    }

    // acceleration to correct velocity
    double accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    // constrain correction acceleration from accel_min to accel_max
    accel_target = constrain_float(accel_target, accel_min, accel_max);

    // velocity correction with input velocity
    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled
    if (limit_total_accel) {
        accel_target = constrain_float(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt);
}

void shape_vel_accel_xy(const Eigen::Vector2d& vel_desired, const Eigen::Vector2d& accel_desired,
                        const Eigen::Vector2d& vel, Eigen::Vector2d& accel,
                        double accel_max, double jerk_max, double dt, bool limit_total_accel)
{
    // sanity check accel_max and jerk_max.
    if (!is_positive(accel_max) || !is_positive(jerk_max)) {
        std::cerr << "[ap_control] shape_vel_accel_xy: invalid limits" << std::endl;
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const double KPa = jerk_max / accel_max;

    // velocity error to be corrected
    const Eigen::Vector2d vel_error = vel_desired - vel;

    // acceleration to correct velocity
    Eigen::Vector2d accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    limit_accel_corner_xy(vel, accel_target, accel_max);

    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled 
    if (limit_total_accel) {
        double accel_norm = accel_target.norm();
        if (accel_norm > accel_max) {
            accel_target *= (accel_max / accel_norm);
        }
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

// ============================================================================
// POSITION-VELOCITY-ACCELERATION SHAPING
// ============================================================================

void shape_pos_vel_accel(postype_t pos_desired, double vel_desired, double accel_desired,
                         postype_t pos, double vel, double& accel,
                         double vel_min, double vel_max,
                         double accel_min, double accel_max,
                         double jerk_max, double dt, bool limit_total)
{
    // Sanity check limits and jerk_max.
    if (is_positive(vel_min) || is_negative(vel_max) || !is_negative(accel_min) || 
        !is_positive(accel_max) || !is_positive(jerk_max)) {
        std::cerr << "[ap_control] shape_pos_vel_accel: invalid limits" << std::endl;
        return;
    }

    // Position error to be corrected.
    const double pos_error = pos_desired - pos;

    // Select sqrt_controller parameters based on error sign
    double accel_lim;
    double k_v;
    if (is_positive(pos_error)) {
        accel_lim = -accel_min;         // acceleration limit magnitude (positive)
        k_v = jerk_max / accel_lim;     // inner velocity-loop gain (1/s)
    } else {
        accel_lim = accel_max;
        k_v = jerk_max / accel_lim;
    }

    // Work in the correction frame by removing the feedforward velocity.
    const double vel_corr = vel - vel_desired;

    // Velocity correction command derived from position error
    double vel_corr_cmd = sqrt_controller(pos_error, k_v, accel_lim, dt);

    // Rate-of-change implied by the shaped velocity correction
    const double accel_corr_cmd = sqrt_controller_accel(pos_error, vel_corr_cmd, vel_corr, k_v, accel_lim);

    // Convert the implied rate-of-change term into an equivalent velocity correction bias
    vel_corr_cmd += accel_corr_cmd / k_v;

    // Limit correction velocity magnitude
    if (is_negative(vel_min)) {
        vel_corr_cmd = std::max(vel_corr_cmd, vel_min);
    }  
    if (is_positive(vel_max)) {
        vel_corr_cmd = std::min(vel_corr_cmd, vel_max);
    }

    // Total velocity target = feedforward + correction.
    double vel_target = vel_desired + vel_corr_cmd;

    // Constrain total velocity if limiting is enabled
    if (limit_total) {
        if (is_negative(vel_min)) {
            vel_target = std::max(vel_target, vel_min);
        }
        if (is_positive(vel_max)) {
            vel_target = std::min(vel_target, vel_max);
        }
    }

    // Acceleration demand from velocity error
    double accel_target = (vel_target - vel) * k_v;

    // Bound acceleration command
    accel_target = constrain_float(accel_target, accel_min, accel_max);

    // Add external acceleration feedforward
    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled
    if (limit_total) {
        accel_target = constrain_float(accel_target, accel_min, accel_max);
    }

    // Jerk-limit acceleration toward accel_target
    shape_accel(accel_target, accel, jerk_max, dt);
}

void shape_pos_vel_accel_xy(const Vector2p& pos_desired, const Eigen::Vector2d& vel_desired, 
                            const Eigen::Vector2d& accel_desired,
                            const Vector2p& pos, const Eigen::Vector2d& vel, Eigen::Vector2d& accel,
                            double vel_max, double accel_max,
                            double jerk_max, double dt, bool limit_total)
{
    // sanity check vel_max, accel_max and jerk_max.
    if (is_negative(vel_max) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        std::cerr << "[ap_control] shape_pos_vel_accel_xy: invalid limits" << std::endl;
        return;
    }

    // inner velocity-loop gain derived from jerk/accel limits (1/s)
    const double k_v = jerk_max / accel_max;

    // Velocity correction vector
    Eigen::Vector2d vel_corr_cmd = Eigen::Vector2d::Zero();

    // Position error to be corrected
    const Eigen::Vector2d pos_error = pos_desired - pos;
    const double pos_error_length = pos_error.norm();
    
    if (is_positive(pos_error_length)) {
        // Correction-frame velocity projected onto the position error direction
        double vel_corr_proj = (vel - vel_desired).dot(pos_error) / pos_error_length;

        // Velocity correction magnitude from square-root position controller
        double vel_corr_cmd_length = sqrt_controller(pos_error_length, k_v, accel_max, dt);

        // Rate-of-change implied by the shaped velocity correction
        double accel_corr_cmd_length = sqrt_controller_accel(pos_error_length, vel_corr_cmd_length, 
                                                              vel_corr_proj, k_v, accel_max);

        // Convert the implied rate-of-change term into an equivalent velocity correction bias
        vel_corr_cmd_length += accel_corr_cmd_length / k_v;

        // Limit correction velocity magnitude
        if (is_positive(vel_max)) {
            vel_corr_cmd_length = constrain_float(vel_corr_cmd_length, -vel_max, vel_max);
        }

        // Map scalar correction back onto the position error direction
        vel_corr_cmd = pos_error * (vel_corr_cmd_length / pos_error_length);
    }

    // Total velocity target is the sum of feedforward velocity and correction
    Eigen::Vector2d vel_target = vel_desired + vel_corr_cmd;
    if (limit_total && is_positive(vel_max)) {
        double vel_norm = vel_target.norm();
        if (vel_norm > vel_max) {
            vel_target *= (vel_max / vel_norm);
        }
    }

    // Acceleration demand from velocity error
    Eigen::Vector2d accel_target = (vel_target - vel) * k_v;

    // Limit acceleration magnitude while prioritising braking
    limit_accel_corner_xy(vel, accel_target, accel_max);

    // Add external acceleration feedforward
    accel_target += accel_desired;

    if (limit_total) {
        double accel_norm = accel_target.norm();
        if (accel_norm > accel_max) {
            accel_target *= (accel_max / accel_norm);
        }
    }

    // Apply jerk limiting to smoothly approach the target acceleration
    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

void shape_angle_vel_accel(double angle_desired, double angle_vel_desired, double angle_accel_desired,
                           double angle, double angle_vel, double& angle_accel,
                           double angle_vel_max, double angle_accel_max,
                           double angle_jerk_max, double dt, bool limit_total)
{
    // Wrap desired angle to the nearest equivalent setpoint relative to the current angle
    const double angle_desired_wrapped = angle + wrap_PI(angle_desired - angle);
    shape_pos_vel_accel(angle_desired_wrapped, angle_vel_desired, angle_accel_desired, 
                        angle, angle_vel, angle_accel, 
                        -angle_vel_max, angle_vel_max, 
                        -angle_accel_max, angle_accel_max, 
                        angle_jerk_max, dt, limit_total); 
}

void shape_pos_vel_accel_xyz(const Eigen::Vector3d& pos_desired, const Eigen::Vector3d& vel_desired, 
                             const Eigen::Vector3d& accel_desired,
                             const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, Eigen::Vector3d& accel,
                             double vel_max_xy, double accel_max_xy, double jerk_max_xy,
                             double vel_max_z, double accel_max_z, double jerk_max_z,
                             double dt, bool limit_total)
{
    // ========================================================================
    // Horizontal (XY) shaping - uses shape_pos_vel_accel_xy
    // ========================================================================
    
    Vector2p pos_xy = pos.head<2>();
    Eigen::Vector2d vel_xy = vel.head<2>();
    Eigen::Vector2d accel_xy = accel.head<2>();
    
    shape_pos_vel_accel_xy(
        pos_desired.head<2>(),      // desired position XY
        vel_desired.head<2>(),      // desired velocity XY
        accel_desired.head<2>(),    // desired acceleration XY
        pos_xy,                     // current position XY
        vel_xy,                     // current velocity XY
        accel_xy,                   // current acceleration XY (output)
        vel_max_xy,                 // velocity limit
        accel_max_xy,               // acceleration limit
        jerk_max_xy,                // jerk limit
        dt,                         // time step
        limit_total                 // limit total
    );
    
    accel.head<2>() = accel_xy;
    
    // ========================================================================
    // Vertical (Z) shaping - uses shape_pos_vel_accel
    // ========================================================================
    
    double accel_z = accel.z();
    
    shape_pos_vel_accel(
        pos_desired.z(),            // desired position Z
        vel_desired.z(),            // desired velocity Z
        accel_desired.z(),          // desired acceleration Z
        pos.z(),                    // current position Z
        vel.z(),                    // current velocity Z
        accel_z,                    // current acceleration Z (output)
        -vel_max_z,                 // vel_min
        vel_max_z,                  // vel_max
        -accel_max_z,               // accel_min
        accel_max_z,                // accel_max
        jerk_max_z,                 // jerk_max
        dt,                         // time step
        limit_total                 // limit total
    );
    
    accel.z() = accel_z;
}

// ============================================================================
// SQUARE-ROOT CONTROLLER
// ============================================================================

double sqrt_controller(double error, double p, double second_ord_lim, double dt)
{
    double correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // No second-order limit: use pure linear controller
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // No P gain, but with acceleration limit — use sqrt-shaped response only
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * error);
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0;
        }
    } else {
        // Both P and second-order limits defined — use hybrid model
        const double linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            // Positive error beyond linear region — use sqrt branch
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)));
        } else if (error < -linear_dist) {
            // Negative error beyond linear region — use sqrt branch
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)));
        } else {
            // Inside linear region
            correction_rate = error * p;
        }
    }
    if (is_positive(dt)) {
        // Clamp to ensure we do not overshoot the error in the last time step
        return constrain_float(correction_rate, -std::abs(error) / dt, std::abs(error) / dt);
    } else {
        return correction_rate;
    }
}

Eigen::Vector2d sqrt_controller(const Eigen::Vector2d& error, double p, double second_ord_lim, double dt)
{
    const double error_length = error.norm();
    if (!is_positive(error_length)) {
        return Eigen::Vector2d::Zero();
    }

    const double correction_length = sqrt_controller(error_length, p, second_ord_lim, dt);
    return error * (correction_length / error_length);
}

double inv_sqrt_controller(double output, double p, double D_max)
{
    // Degenerate case: second-order limit (D_max) is positive, but P gain is zero
    if (is_positive(D_max) && is_zero(p)) {
        return (output * output) / (2.0 * D_max);
    }

    // Degenerate case: no D_max, but P gain is non-zero → use linear model
    if ((is_negative(D_max) || is_zero(D_max)) && !is_zero(p)) {
        return output / p;
    }

    // Degenerate case: both gains are zero — no useful model
    if ((is_negative(D_max) || is_zero(D_max)) && is_zero(p)) {
        return 0.0;
    }

    // Compute transition threshold between linear and sqrt regions
    const double linear_velocity = D_max / p;

    if (std::abs(output) < linear_velocity) {
        // Linear region: below transition threshold
        return output / p;
    }

    // Square-root region: above transition threshold
    const double linear_dist = D_max / sq(p);
    const double stopping_dist = (linear_dist * 0.5) + sq(output) / (2.0 * D_max);
    return is_positive(output) ? stopping_dist : -stopping_dist;
}

double sqrt_controller_accel(double error, double rate_cmd, double rate_state, double p, double second_ord_lim)
{
    // If we are moving away from the target return zero.
    if (!is_positive(rate_cmd * rate_state)) {
        return 0.0;
    }

    // If no second-order limit, controller is linear everywhere (rate_cmd ~ p*error).
    if (!is_positive(second_ord_lim)) {
        return -p * rate_state;
    }

    // If no P gain but second-order limit exists, controller is pure sqrt everywhere.
    if (!is_positive(p)) {
        if (is_zero(rate_cmd)) {
            return 0.0;
        }
        return -(second_ord_lim / std::abs(rate_cmd)) * rate_state;
    }

    // Both P and second-order limit defined — match sqrt_controller() region selection.
    const double linear_dist = second_ord_lim / sq(p);

    if (std::abs(error) <= linear_dist) {
        // Inside linear region.
        return -p * rate_state;
    }

    // Outside linear region (sqrt branch). Guard divide-by-zero on rate_cmd.
    if (is_zero(rate_cmd)) {
        return 0.0;
    }
    return -(second_ord_lim / std::abs(rate_cmd)) * rate_state;
}

double stopping_distance(double velocity, double p, double accel_max)
{
    return inv_sqrt_controller(velocity, p, accel_max);
}

// ============================================================================
// ACCELERATION LIMITING
// ============================================================================

bool limit_accel_xy(const Eigen::Vector2d& vel, Eigen::Vector2d& accel, double accel_max)
{
    // check accel_max is defined
    if (!is_positive(accel_max)) {
        return false;
    }
    // limit acceleration to accel_max while prioritizing cross track acceleration
    if (accel.squaredNorm() > sq(accel_max)) {
        if (is_zero(vel.squaredNorm())) {
            // We do not have a direction of travel so do a simple vector length limit
            double accel_norm = accel.norm();
            if (accel_norm > accel_max) {
                accel *= (accel_max / accel_norm);
            }
        } else {
            // calculate acceleration in the direction of and perpendicular to the velocity input
            const Eigen::Vector2d vel_unit = vel.normalized();
            // acceleration in the direction of travel
            double accel_dir = vel_unit.dot(accel);
            // cross track acceleration
            Eigen::Vector2d accel_cross = accel - (vel_unit * accel_dir);
            double accel_cross_norm = accel_cross.norm();
            if (accel_cross_norm > accel_max) {
                accel_cross *= (accel_max / accel_cross_norm);
                accel_dir = 0.0;
            } else {
                double accel_max_dir = safe_sqrt(sq(accel_max) - accel_cross.squaredNorm());
                accel_dir = constrain_float(accel_dir, -accel_max_dir, accel_max_dir);
            }
            accel = accel_cross + vel_unit * accel_dir;
        }
        return true;
    }
    return false;
}

bool limit_accel_corner_xy(const Eigen::Vector2d& vel, Eigen::Vector2d& accel, double accel_max)
{
    // Check accel_max is defined.
    if (!is_positive(accel_max)) {
        return false;
    }

    if (is_zero(vel.squaredNorm())) {
        // No along/cross decomposition possible; apply a simple magnitude limit.
        double accel_norm = accel.norm();
        if (accel_norm > accel_max) {
            accel *= (accel_max / accel_norm);
            return true;
        }
        return false;
    }

    // Pre-limit to keep the acceleration direction well-conditioned.
    double accel_norm = accel.norm();
    if (accel_norm > 2.0 * accel_max) {
        accel *= (2.0 * accel_max / accel_norm);
    }

    // Unit velocity direction defines the along-track axis.
    const Eigen::Vector2d vel_unit = vel.normalized();

    // Signed scalar projection of acceleration onto the velocity direction.
    double accel_dir_scalar = accel.dot(vel_unit);

    // Along-track and cross-track acceleration components.
    Eigen::Vector2d accel_dir = vel_unit * accel_dir_scalar;
    Eigen::Vector2d accel_cross = accel - accel_dir;

    if (is_positive(accel_dir_scalar)) {
        // Non-braking regime
        // Prioritise cross-track acceleration and allocate the remaining budget to along-track.

        // Limit cross-track magnitude first.
        const double accel_cross_mag = std::min(accel_cross.norm(), accel_max);
        const double accel_along_max = safe_sqrt(sq(accel_max) - sq(accel_cross_mag));

        double cross_norm = accel_cross.norm();
        if (cross_norm > accel_max) {
            accel_cross *= (accel_max / cross_norm);
        }
        double dir_norm = accel_dir.norm();
        if (dir_norm > accel_along_max) {
            accel_dir *= (accel_along_max / dir_norm);
        }

        accel = accel_cross + accel_dir;
        return true;
    }

    // Braking regime
    // Prioritise along-track deceleration and allocate the remaining budget to cross-track.

    // Limit braking magnitude.
    accel_dir_scalar = std::max(accel_dir_scalar, -accel_max);
    accel_dir = vel_unit * accel_dir_scalar;

    // Allocate remaining acceleration budget to cross-track.
    const double accel_cross_max = safe_sqrt(sq(accel_max) - sq(accel_dir_scalar));
    double cross_norm = accel_cross.norm();
    if (cross_norm > accel_cross_max) {
        accel_cross *= (accel_cross_max / cross_norm);
    }

    accel = accel_cross + accel_dir;
    return true;
}

// ============================================================================
// KINEMATIC LIMIT CALCULATIONS
// ============================================================================

double kinematic_limit(const Eigen::Vector3d& direction, double max_xy, double max_z_neg, double max_z_pos)
{
    // Reject zero-length direction vectors or undefined limits
    if (is_zero(direction.squaredNorm())) {
        return 0.0;
    }

    const double segment_length_xy = direction.head<2>().norm();
    
    return kinematic_limit(segment_length_xy, direction.z(), max_xy, max_z_neg, max_z_pos);
}

double kinematic_limit(double segment_length_xy, double segment_length_z, 
                       double max_xy, double max_z_neg, double max_z_pos)
{
    // Reject zero-length direction vectors or undefined limits
    if (is_zero(max_xy) || is_zero(max_z_pos) || is_zero(max_z_neg)) {
        return 0.0;
    }

    max_xy = std::abs(max_xy);
    max_z_pos = std::abs(max_z_pos);
    max_z_neg = std::abs(max_z_neg);

    const double length = safe_sqrt(sq(segment_length_xy) + sq(segment_length_z));
    // check for divide by zero.
    if (!is_positive(length)) {
        return 0.0;
    }
    segment_length_xy /= length;
    segment_length_z /= length;

    if (is_zero(segment_length_xy)) {
        // Pure vertical motion
        return is_positive(segment_length_z) ? max_z_pos : max_z_neg;
    }

    if (is_zero(segment_length_z)) {
        // Pure horizontal motion
        return max_xy;
    }

    // Compute vertical-to-horizontal slope of desired direction
    const double slope = segment_length_z / segment_length_xy;
    if (is_positive(slope)) {
        // Ascending: check if slope is within limits
        if (std::abs(slope) < max_z_pos / max_xy) {
            return max_xy / segment_length_xy;
        }
        // Vertical limit dominates in upward direction
        return std::abs(max_z_pos / segment_length_z);
    }

    // Descending: check if slope is within limits
    if (std::abs(slope) < max_z_neg / max_xy) {
        return max_xy / segment_length_xy;
    }

    // Vertical limit dominates in downward direction
    return std::abs(max_z_neg / segment_length_z);
}

// ============================================================================
// INPUT SHAPING & CONVERSION UTILITIES
// ============================================================================

double input_expo(double input, double expo)
{
    // Clamp input to normalized stick range
    input = constrain_float(input, -1.0, 1.0);
    if (expo < 0.95) {
        // Expo shaping: increases control around center stick
        return (1.0 - expo) * input / (1.0 - expo * std::abs(input));
    }
    // If expo is too close to 1, return input unchanged
    return input;
}

double angle_rad_to_accel_mss(double angle_rad)
{
    // Convert lean angle to horizontal acceleration
    return GRAVITY_MSS * std::tan(angle_rad);
}

double angle_deg_to_accel_mss(double angle_deg)
{
    // Convert degrees to radians, then to acceleration
    return angle_rad_to_accel_mss(radians(angle_deg));
}

double accel_mss_to_angle_rad(double accel_mss)
{
    // Inverse of angle_rad_to_accel_mss
    return std::atan(accel_mss / GRAVITY_MSS);
}

double accel_mss_to_angle_deg(double accel_mss)
{
    // Convert result of radian-based conversion to degrees
    return degrees(accel_mss_to_angle_rad(accel_mss));
}

void rc_input_to_roll_pitch_rad(double roll_in_norm, double pitch_in_norm, 
                                double angle_max_rad, double angle_limit_rad, 
                                double& roll_out_rad, double& pitch_out_rad)
{
    // Constrain angle_max to 85 deg to avoid unstable behavior
    angle_max_rad = std::min(angle_max_rad, radians(85.0));

    // Convert normalized pitch and roll stick input into horizontal thrust components
    Eigen::Vector2d thrust;
    thrust.x() = -std::tan(angle_max_rad * pitch_in_norm);
    thrust.y() = std::tan(angle_max_rad * roll_in_norm);

    // Calculate the horizontal thrust limit based on angle limit
    angle_limit_rad = constrain_float(angle_limit_rad, radians(10.0), angle_max_rad);
    double thrust_limit = std::tan(angle_limit_rad);

    // Apply limit to the horizontal thrust vector (preserves stick direction)
    double thrust_norm = thrust.norm();
    if (thrust_norm > thrust_limit) {
        thrust *= (thrust_limit / thrust_norm);
    }

    // Convert thrust vector back to pitch and roll Euler angles
    pitch_out_rad = -std::atan(thrust.x());
    roll_out_rad = std::atan(std::cos(pitch_out_rad) * thrust.y());
}

}  // namespace ap_control
