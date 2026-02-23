#ifndef DRONE_OFFBOARD_CORE_ESTIMATOR_TARGET_ESTIMATOR_HPP_
#define DRONE_OFFBOARD_CORE_ESTIMATOR_TARGET_ESTIMATOR_HPP_

/**
 * @file target_estimator.hpp
 * @brief Target GPS processing and state estimation (NO ROS DEPENDENCY)
 * 
 * This class encapsulates all target GPS processing:
 * - Jitter correction
 * - DT validation
 * - Alpha-Beta filtering
 * - Velocity/acceleration estimation
 * - Heading calculation
 * - GPS to ENU conversion
 * 
 * Usage:
 *   TargetEstimator estimator(params, origin_manager);
 *   
 *   GPSMeasurement meas;
 *   meas.lat = ...;
 *   meas.lon = ...;
 *   meas.alt = ...;
 *   meas.timestamp_sec = ...;
 *   
 *   if (estimator.update(meas)) {
 *       const TargetState& state = estimator.state();
 *       // Use state...
 *   }
 */

#include <Eigen/Dense>
#include <optional>
#include "../types/target_state.hpp"
#include "../types/common.hpp"
#include "../types/parameters.hpp"

namespace drone_follow {
namespace core {

// Forward declarations
class OriginManager;
class DtValidator;
class JitterCorrection;

/**
 * @brief Alpha-Beta Filter for position and velocity estimation
 */
class AlphaBetaFilter {
public:
    double x = 0.0;  // Estimated position
    double v = 0.0;  // Estimated velocity
    
    /**
     * @brief Standard update with fixed beta
     */
    void update(double measurement, double dt, double alpha, double beta);
    
    /**
     * @brief Adaptive beta update (faster response when residual is large)
     */
    void update_adaptive(double measurement, double dt, double alpha,
                        double beta_fast, double beta_slow, double residual_threshold);
    
    /**
     * @brief Reset filter state
     */
    void reset(double measurement);
};

/**
 * @brief Origin manager for GPS to ENU conversion
 */
class OriginManager {
public:
    OriginManager() : ready_(false) {}
    
    /**
     * @brief Set origin (only once)
     * @return true if origin was set, false if already set
     */
    bool set_origin(double lat, double lon, double alt);
    
    /**
     * @brief Check if origin is ready
     */
    bool is_ready() const { return ready_; }
    
    /**
     * @brief Get origin values
     */
    bool get_origin(double& lat, double& lon, double& alt) const;
    
    /**
     * @brief Convert GPS to Map-ENU
     */
    bool gps_to_enu(double lat, double lon, double& east, double& north) const;
    
private:
    bool ready_;
    double lat_ = 0.0;
    double lon_ = 0.0;
    double alt_ = 0.0;
};

/**
 * @brief DT validator
 */
class DtValidator {
public:
    struct ValidationResult {
        double dt;
        bool valid;
        bool should_reset;
    };
    
    DtValidator(double min_dt, double max_dt, double default_dt);
    
    ValidationResult validate(double current_time);
    void reset();
    double get_last_valid_time() const { return last_valid_time_; }
    
private:
    double min_dt_;
    double max_dt_;
    double default_dt_;
    double last_valid_time_;
    int valid_count_;
};

/**
 * @brief Jitter correction for timestamp
 */
class JitterCorrection {
public:
    JitterCorrection(int max_lag_ms, int convergence_loops);
    
    uint64_t correct_offboard_timestamp_usec(uint64_t offboard_usec, uint64_t local_usec);
    
private:
    int max_lag_ms_;
    int convergence_loops_;
    int64_t lag_sum_ = 0;
    int sample_count_ = 0;
    bool converged_ = false;
};

/**
 * @brief Target estimator - processes GPS measurements and estimates state
 */
class TargetEstimator {
public:
    /**
     * @brief Constructor
     * @param params Estimator parameters
     * @param origin Origin manager for GPS conversion
     */
    TargetEstimator(const EstimatorParams& params, OriginManager& origin);
    
    /**
     * @brief Update with new GPS measurement
     * @param meas GPS measurement
     * @param local_time_sec Local system time (for jitter correction)
     * @return true if update successful and state is valid
     */
    bool update(const GPSMeasurement& meas, double local_time_sec);
    
    /**
     * @brief Get current estimated state
     */
    const TargetState& state() const { return state_; }
    
    /**
     * @brief Reset estimator
     */
    void reset();
    
    /**
     * @brief Get last estimate for error checking
     */
    const Eigen::Vector3d& last_estimate_pos() const { return last_estimate_pos_; }
    const Eigen::Vector3d& last_estimate_vel() const { return last_estimate_vel_; }
    
private:
    // Parameters
    EstimatorParams params_;
    
    // Origin manager reference
    OriginManager& origin_;
    
    // State
    TargetState state_;
    
    // Filters
    AlphaBetaFilter filter_vel_e_;
    AlphaBetaFilter filter_vel_n_;
    AlphaBetaFilter filter_alt_;
    
    // Validation
    DtValidator dt_validator_;
    std::optional<JitterCorrection> jitter_correction_;
    
    // Last estimate for error checking
    Eigen::Vector3d last_estimate_pos_;
    Eigen::Vector3d last_estimate_vel_;
    
    /**
     * @brief Correct timestamp using jitter correction
     */
    double correct_timestamp(double offboard_sec, double local_sec, bool offboard_valid);
    
    /**
     * @brief Convert GPS to ENU and update position
     */
    bool convert_to_enu(double lat, double lon, double alt, Eigen::Vector2d& pos_xy);
    
    /**
     * @brief Estimate velocity from position change
     */
    void estimate_velocity(const Eigen::Vector2d& new_pos, double dt);
    
    /**
     * @brief Estimate acceleration from velocity change
     */
    void estimate_acceleration(double dt);
    
    /**
     * @brief Calculate heading from velocity
     */
    void calculate_heading();
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_ESTIMATOR_TARGET_ESTIMATOR_HPP_
