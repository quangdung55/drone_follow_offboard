/**
 * @file target_estimator.cpp
 * @brief Target GPS processing and state estimation implementation
 */

#include "drone_offboard/core/estimator/target_estimator.hpp"
#include <cmath>
#include <algorithm>

namespace drone_follow {
namespace core {

// ============================================================================
// ALPHA-BETA FILTER
// ============================================================================

void AlphaBetaFilter::update(double measurement, double dt, double alpha, double beta) {
    if (dt <= 0.0) return;
    
    // Prediction step
    double x_pred = x + v * dt;
    
    // Update step
    double residual = measurement - x_pred;
    x = x_pred + alpha * residual;
    v = v + (beta * residual) / dt;
}

void AlphaBetaFilter::update_adaptive(double measurement, double dt, double alpha,
                                      double beta_fast, double beta_slow, 
                                      double residual_threshold) {
    if (dt <= 0.0) return;
    
    // Prediction step
    double x_pred = x + v * dt;
    
    // Update step with adaptive beta
    double residual = measurement - x_pred;
    double abs_r = std::abs(residual);
    double beta_eff = (abs_r > residual_threshold) ? beta_fast : beta_slow;
    
    x = x_pred + alpha * residual;
    v = v + (beta_eff * residual) / dt;
}

void AlphaBetaFilter::reset(double measurement) {
    x = measurement;
    v = 0.0;
}

// ============================================================================
// ORIGIN MANAGER
// ============================================================================

bool OriginManager::set_origin(double lat, double lon, double alt) {
    if (ready_) {
        return false; // Already set
    }
    lat_ = lat;
    lon_ = lon;
    alt_ = alt;
    ready_ = true;
    return true;
}

bool OriginManager::get_origin(double& lat, double& lon, double& alt) const {
    if (!ready_) {
        return false;
    }
    lat = lat_;
    lon = lon_;
    alt = alt_;
    return true;
}

bool OriginManager::gps_to_enu(double lat, double lon, double& east, double& north) const {
    double origin_lat, origin_lon, origin_alt;
    if (!get_origin(origin_lat, origin_lon, origin_alt)) {
        return false;
    }
    
    double d_lat = (lat - origin_lat) * DEG_TO_RAD;
    double d_lon = (lon - origin_lon) * DEG_TO_RAD;
    double lat0_rad = origin_lat * DEG_TO_RAD;
    
    north = d_lat * R_EARTH;
    east = d_lon * R_EARTH * std::cos(lat0_rad);
    
    return true;
}

// ============================================================================
// DT VALIDATOR
// ============================================================================

DtValidator::DtValidator(double min_dt, double max_dt, double default_dt)
    : min_dt_(min_dt), max_dt_(max_dt), default_dt_(default_dt),
      last_valid_time_(0.0), valid_count_(0) {}

DtValidator::ValidationResult DtValidator::validate(double current_time) {
    ValidationResult result;
    
    if (last_valid_time_ == 0.0) {
        // First call - initialize
        last_valid_time_ = current_time;
        result.dt = default_dt_;
        result.valid = false;
        result.should_reset = false;
        return result;
    }
    
    double dt = current_time - last_valid_time_;
    
    // Duplicate timestamp
    if (dt == 0.0) {
        result.dt = default_dt_;
        result.valid = false;
        result.should_reset = false;
        return result;
    }
    
    if (dt < min_dt_ || dt > max_dt_) {
        // Invalid dt detected
        result.dt = default_dt_;
        result.valid = false;
        result.should_reset = (dt > max_dt_); // Reset ONLY if timeout
        return result;
    }
    
    // Valid dt
    last_valid_time_ = current_time;
    result.dt = dt;
    result.valid = true;
    result.should_reset = false;
    valid_count_++;
    
    return result;
}

void DtValidator::reset() {
    last_valid_time_ = 0.0;
    valid_count_ = 0;
}

// ============================================================================
// JITTER CORRECTION
// ============================================================================

JitterCorrection::JitterCorrection(int max_lag_ms, int convergence_loops)
    : max_lag_ms_(max_lag_ms), convergence_loops_(convergence_loops) {}

uint64_t JitterCorrection::correct_offboard_timestamp_usec(uint64_t offboard_usec, 
                                                           uint64_t local_usec) {
    // Calculate lag
    int64_t lag_usec = static_cast<int64_t>(local_usec) - static_cast<int64_t>(offboard_usec);
    
    // Reject unreasonable lags
    int64_t max_lag_usec = max_lag_ms_ * 1000LL;
    if (lag_usec < 0 || lag_usec > max_lag_usec) {
        // Reset convergence
        converged_ = false;
        lag_sum_ = 0;
        sample_count_ = 0;
        return local_usec; // Use local time
    }
    
    // Accumulate samples
    lag_sum_ += lag_usec;
    sample_count_++;
    
    // Check convergence
    if (!converged_ && sample_count_ >= convergence_loops_) {
        converged_ = true;
    }
    
    if (!converged_) {
        return local_usec; // Not converged yet, use local time
    }
    
    // Apply average lag correction
    int64_t avg_lag = lag_sum_ / sample_count_;
    uint64_t corrected = offboard_usec + avg_lag;
    
    return corrected;
}

// ============================================================================
// TARGET ESTIMATOR
// ============================================================================

TargetEstimator::TargetEstimator(const EstimatorParams& params, OriginManager& origin)
    : params_(params),
      origin_(origin),
      dt_validator_(GPS_DT_MIN_VALID, GPS_DT_MAX_VALID, CONTROL_LOOP_DT_DEFAULT),
      last_estimate_pos_(Eigen::Vector3d::Zero()),
      last_estimate_vel_(Eigen::Vector3d::Zero()) {
    
    // Initialize jitter correction if enabled
    if (params_.jitter_correction_enable) {
        jitter_correction_.emplace(params_.jitter_max_lag_ms, 
                                   params_.jitter_convergence_loops);
    }
    
    // Initialize filters
    filter_vel_e_.reset(0.0);
    filter_vel_n_.reset(0.0);
    filter_alt_.reset(0.0);
}

bool TargetEstimator::update(const GPSMeasurement& meas, double local_time_sec) {
    // Check origin ready
    if (!origin_.is_ready()) {
        return false;
    }
    
    // === TIMESTAMP JITTER CORRECTION ===
    bool offboard_valid = (meas.timestamp_sec > 0) && 
                         (meas.timestamp_sec < local_time_sec + 1.0);
    
    double corrected_time = correct_timestamp(meas.timestamp_sec, local_time_sec, 
                                              offboard_valid);
    
    // === DT VALIDATION ===
    auto dt_result = dt_validator_.validate(corrected_time);
    
    // Handle reset (timeout)
    if (dt_result.should_reset) {
        reset();
        return false;
    }
    
    // Skip if dt invalid (duplicate or first update)
    if (!dt_result.valid) {
        return false;
    }
    
    double dt = dt_result.dt;
    
    // === GPS TO ENU CONVERSION ===
    Eigen::Vector2d new_pos_xy;
    if (!convert_to_enu(meas.lat, meas.lon, meas.alt, new_pos_xy)) {
        return false;
    }
    
    // === VELOCITY ESTIMATION ===
    if (state_.valid) {
        estimate_velocity(new_pos_xy, dt);
        estimate_acceleration(dt);
        calculate_heading();
        
        // Update altitude with filter
        double origin_lat, origin_lon, origin_alt;
        origin_.get_origin(origin_lat, origin_lon, origin_alt);
        
        if (params_.filter_enable) {
            filter_alt_.update(meas.alt, dt, params_.filter_alpha, params_.filter_beta);
            state_.alt = filter_alt_.x;
            state_.pos_enu.z() = filter_alt_.x - origin_alt;
        } else {
            state_.alt = meas.alt;
            state_.pos_enu.z() = meas.alt - origin_alt;
        }
    } else {
        // First valid update - initialize filters
        filter_vel_e_.reset(0.0);
        filter_vel_n_.reset(0.0);
        filter_alt_.reset(meas.alt);
        
        double origin_lat, origin_lon, origin_alt;
        origin_.get_origin(origin_lat, origin_lon, origin_alt);
        state_.pos_enu.z() = meas.alt - origin_alt;
        state_.alt = meas.alt;
    }
    
    // Update position
    state_.pos_enu.x() = new_pos_xy.x();
    state_.pos_enu.y() = new_pos_xy.y();
    state_.lat = meas.lat;
    state_.lon = meas.lon;
    state_.last_update_sec = corrected_time;
    state_.valid = true;
    
    // Save for error checking
    last_estimate_pos_ = state_.pos_enu;
    last_estimate_vel_ = state_.vel_enu;
    
    return true;
}

void TargetEstimator::reset() {
    filter_vel_e_.reset(0.0);
    filter_vel_n_.reset(0.0);
    filter_alt_.reset(0.0);
    state_.reset();
    dt_validator_.reset();
    last_estimate_pos_.setZero();
    last_estimate_vel_.setZero();
}

double TargetEstimator::correct_timestamp(double offboard_sec, double local_sec, 
                                          bool offboard_valid) {
    if (!params_.jitter_correction_enable || !offboard_valid || !jitter_correction_.has_value()) {
        return local_sec;
    }
    
    uint64_t offboard_usec = static_cast<uint64_t>(offboard_sec * 1e6);
    uint64_t local_usec = static_cast<uint64_t>(local_sec * 1e6);
    
    uint64_t corrected_usec = jitter_correction_->correct_offboard_timestamp_usec(
        offboard_usec, local_usec);
    
    // Validate corrected timestamp
    if (corrected_usec == 0 || corrected_usec > local_usec + 1000000ULL) {
        return local_sec;
    }
    
    return corrected_usec / 1e6;
}

bool TargetEstimator::convert_to_enu(double lat, double lon, double alt, 
                                     Eigen::Vector2d& pos_xy) {
    double east, north;
    if (!origin_.gps_to_enu(lat, lon, east, north)) {
        return false;
    }
    
    pos_xy.x() = east;
    pos_xy.y() = north;
    return true;
}

void TargetEstimator::estimate_velocity(const Eigen::Vector2d& new_pos, double dt) {
    // Calculate raw velocity
    Eigen::Vector2d delta = new_pos - state_.pos_enu.head<2>();
    Eigen::Vector2d raw_vel = delta / dt;
    
    // Check reasonableness
    double raw_speed = raw_vel.norm();
    if (raw_speed > MAX_TARGET_VELOCITY) {
        return; // Reject unreasonable velocity
    }
    
    double old_vel_e = state_.vel_enu.x();
    double old_vel_n = state_.vel_enu.y();
    
    // Apply filter
    if (params_.filter_enable) {
        filter_vel_e_.update_adaptive(raw_vel.x(), dt, params_.filter_alpha,
                                     params_.filter_beta_fast,
                                     params_.filter_beta_slow,
                                     params_.filter_residual_threshold);
        filter_vel_n_.update_adaptive(raw_vel.y(), dt, params_.filter_alpha,
                                     params_.filter_beta_fast,
                                     params_.filter_beta_slow,
                                     params_.filter_residual_threshold);
        
        state_.vel_enu.x() = filter_vel_e_.x;
        state_.vel_enu.y() = filter_vel_n_.x;
    } else {
        state_.vel_enu.x() = raw_vel.x();
        state_.vel_enu.y() = raw_vel.y();
    }
}

void TargetEstimator::estimate_acceleration(double dt) {
    static double old_vel_e = 0.0;
    static double old_vel_n = 0.0;
    
    // Calculate instantaneous acceleration
    double instant_accel_e = (state_.vel_enu.x() - old_vel_e) / dt;
    double instant_accel_n = (state_.vel_enu.y() - old_vel_n) / dt;
    
    // Low-pass filter
    state_.accel_enu.x() = ACCEL_FILTER_ALPHA * instant_accel_e +
                          ACCEL_FILTER_BETA * state_.accel_enu.x();
    state_.accel_enu.y() = ACCEL_FILTER_ALPHA * instant_accel_n +
                          ACCEL_FILTER_BETA * state_.accel_enu.y();
    
    // Clamp acceleration
    double accel_mag = state_.accel_enu.head<2>().norm();
    if (accel_mag > ACCEL_MAGNITUDE_CLAMP) {
        state_.accel_enu.head<2>() *= (ACCEL_MAGNITUDE_CLAMP / accel_mag);
    }
    
    old_vel_e = state_.vel_enu.x();
    old_vel_n = state_.vel_enu.y();
}

void TargetEstimator::calculate_heading() {
    double speed = state_.vel_enu.head<2>().norm();
    if (speed > SPEED_THRESHOLD_HEADING) {
        double new_heading = std::atan2(state_.vel_enu.y(), state_.vel_enu.x());
        
        // Calculate angular difference
        double diff_heading = new_heading - state_.heading_rad;
        while (diff_heading > M_PI) diff_heading -= 2.0 * M_PI;
        while (diff_heading < -M_PI) diff_heading += 2.0 * M_PI;
        
        // Estimate yaw rate (need dt from last call)
        
        // For now, store heading
        state_.heading_rad = new_heading;
        // TODO: Calculate yaw rate properly with dt tracking
    }
}

} // namespace core
} // namespace drone_follow
