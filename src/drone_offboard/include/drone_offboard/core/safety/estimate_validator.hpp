#ifndef DRONE_OFFBOARD_CORE_SAFETY_ESTIMATE_VALIDATOR_HPP_
#define DRONE_OFFBOARD_CORE_SAFETY_ESTIMATE_VALIDATOR_HPP_

/**
 * @file estimate_validator.hpp
 * @brief Estimate error validation (NO ROS DEPENDENCY)
 * 
 * Validates target state estimates to detect:
 * - GPS glitches (large position jumps)
 * - Unreasonable velocities/accelerations
 * - Filter divergence
 * 
 * Based on AP_Follow algorithm.
 */

#include <Eigen/Dense>
#include "../types/target_state.hpp"
#include "../types/parameters.hpp"

namespace drone_follow {
namespace core {

/**
 * @brief Validates target state estimates
 */
class EstimateValidator {
public:
    /**
     * @brief Constructor
     * @param params Estimator parameters (for timeout and limits)
     */
    explicit EstimateValidator(const EstimatorParams& params);
    
    /**
     * @brief Check if estimate error is too large
     * @param current_state Current target state
     * @param last_estimate_pos Last estimated position (for comparison)
     * @param last_estimate_vel Last estimated velocity (for comparison)
     * @return true if error is too large (should reset filters)
     */
    bool is_error_too_large(const TargetState& current_state,
                           const Eigen::Vector3d& last_estimate_pos,
                           const Eigen::Vector3d& last_estimate_vel) const;
    
    /**
     * @brief Update last estimate for next check
     */
    void update_last_estimate(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel);
    
private:
    EstimatorParams params_;
    
    /**
     * @brief Calculate maximum velocity change over timeout period
     */
    double calc_max_velocity_change(double accel_max, double jerk_max, 
                                    double timeout_sec) const;
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_SAFETY_ESTIMATE_VALIDATOR_HPP_
