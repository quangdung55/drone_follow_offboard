#pragma once

namespace drone_follow {
namespace core {

/**
 * @brief Emergency action controller (NO ROS dependency)
 * 
 * Provides safety actions like emergency descent
 */
class EmergencyController {
public:
    EmergencyController(double max_altitude_agl, double max_altitude_msl, 
                       double descent_rate_mps)
        : max_alt_agl_(max_altitude_agl),
          max_alt_msl_(max_altitude_msl),
          descent_rate_(descent_rate_mps),
          active_(false) {}
    
    /**
     * @brief Check if altitude is safe
     */
    bool is_altitude_safe(double alt_relative_m, double alt_msl_m) const {
        return alt_relative_m <= max_alt_agl_ && alt_msl_m <= max_alt_msl_;
    }
    
    /**
     * @brief Get emergency descent command
     * @return Vertical velocity command (negative = down)
     */
    double get_descent_command() {
        active_ = true;
        return -descent_rate_;
    }
    
    /**
     * @brief Reset emergency state
     */
    void reset() {
        active_ = false;
    }
    
    /**
     * @brief Check if emergency descent is active
     */
    bool is_active() const {
        return active_;
    }
    
private:
    double max_alt_agl_;   // Max altitude above ground level
    double max_alt_msl_;   // Max altitude mean sea level
    double descent_rate_;  // Descent rate in m/s
    bool active_;
};

} // namespace core
} // namespace drone_follow
