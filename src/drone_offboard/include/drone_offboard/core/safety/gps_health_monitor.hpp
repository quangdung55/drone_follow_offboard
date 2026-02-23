#pragma once

#include <cmath>

namespace drone_follow {
namespace core {

/**
 * @brief GPS health data structure (NO ROS dependency)
 */
struct GPSHealthData {
    double horizontal_accuracy = 999.0;
    double covariance = 999.0;
    double last_update_sec = 0.0;
    bool frozen_detected = false;
    bool has_covariance = false;
    int satellites = 0;
};

/**
 * @brief GPS Health Monitor - All GPS health checking logic
 * 
 * Pure algorithm, no ROS dependencies
 */
class GPSHealthMonitor {
public:
    GPSHealthMonitor(double warn_threshold_m2, double max_threshold_m2, double frozen_timeout_sec)
        : warn_threshold_(warn_threshold_m2),
          max_threshold_(max_threshold_m2),
          frozen_timeout_(frozen_timeout_sec) {}
    
    /**
     * @brief Update health data from covariance
     */
    void update_from_covariance(double cov_x, double cov_y, bool has_covariance, 
                                GPSHealthData& health) const {
        if (has_covariance) {
            health.covariance = std::max(cov_x, cov_y);
            health.horizontal_accuracy = std::sqrt(health.covariance);
            health.has_covariance = true;
        } else {
            health.has_covariance = false;
        }
        health.frozen_detected = false;
    }
    
    /**
     * @brief Check if GPS stream is frozen
     */
    bool is_frozen(double current_time_sec, double last_update_sec) const {
        return (current_time_sec - last_update_sec) > frozen_timeout_;
    }
    
    /**
     * @brief Check if GPS is healthy (meets quality threshold)
     */
    bool is_healthy(const GPSHealthData& health) const {
        if (health.frozen_detected) return false;
        if (!health.has_covariance) return true;  // No data, assume OK
        return health.covariance < max_threshold_;
    }
    
    /**
     * @brief Check if GPS needs warning
     */
    bool needs_warning(const GPSHealthData& health) const {
        if (!health.has_covariance) return false;
        return health.covariance > warn_threshold_;
    }
    
private:
    double warn_threshold_;
    double max_threshold_;
    double frozen_timeout_;
};

} // namespace core
} // namespace drone_follow
