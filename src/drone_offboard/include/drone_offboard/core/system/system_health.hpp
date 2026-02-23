#pragma once

#include "../safety/gps_health_monitor.hpp"
#include "../safety/emergency_controller.hpp"
#include <vector>
#include <string>

namespace drone_follow {
namespace core {

/**
 * @brief System health status report (NO ROS dependency)
 * 
 * Core tính toán health, ROS chỉ convert sang DiagnosticStatus
 */
struct SystemHealthReport {
    enum class Level {
        OK,
        WARN,
        ERROR
    };
    
    Level level = Level::OK;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    
    // State summary
    bool origin_ready = false;
    bool drone_gps_valid = false;
    bool target_valid = false;
    bool altitude_safe = false;
    bool armed = false;
    bool guided = false;
    
    // Metrics
    double drone_gps_accuracy = 999.0;
    double target_gps_accuracy = 999.0;
    double drone_altitude_rel = 0.0;
    double drone_altitude_msl = 0.0;
    double target_timeout_sec = 0.0;
    bool emergency_active = false;
    
    void add_error(const std::string& msg) {
        errors.push_back(msg);
        level = Level::ERROR;
    }
    
    void add_warning(const std::string& msg) {
        warnings.push_back(msg);
        if (level != Level::ERROR) {
            level = Level::WARN;
        }
    }
};

/**
 * @brief Health monitor - Evaluates system health
 */
class SystemHealthMonitor {
public:
    SystemHealthMonitor(GPSHealthMonitor* gps_monitor, EmergencyController* emergency)
        : gps_monitor_(gps_monitor), emergency_(emergency) {}
    
    struct Input {
        bool origin_ready;
        bool drone_gps_valid;
        GPSHealthData drone_gps_health;
        bool target_valid;
        GPSHealthData target_gps_health;
        double drone_alt_rel;
        double drone_alt_msl;
        double time_since_target;
        double timeout_threshold;
        bool armed;
        bool guided;
    };
    
    SystemHealthReport evaluate(const Input& input) {
        SystemHealthReport report;
        
        // Origin check
        report.origin_ready = input.origin_ready;
        if (!input.origin_ready) {
            report.add_error("EKF origin not set");
        }
        
        // Drone GPS check
        report.drone_gps_valid = input.drone_gps_valid;
        report.drone_gps_accuracy = input.drone_gps_health.horizontal_accuracy;
        
        if (!input.drone_gps_valid) {
            report.add_error("Drone GPS invalid");
        } else if (!gps_monitor_->is_healthy(input.drone_gps_health)) {
            report.add_warning("Drone GPS accuracy degraded");
        }
        
        // Target GPS check
        report.target_valid = input.target_valid;
        report.target_gps_accuracy = input.target_gps_health.horizontal_accuracy;
        
        if (!input.target_valid) {
            report.add_warning("No target GPS data");
        } else if (!gps_monitor_->is_healthy(input.target_gps_health)) {
            report.add_warning("Target GPS accuracy degraded");
        }
        
        // Altitude safety
        report.altitude_safe = emergency_->is_altitude_safe(input.drone_alt_rel, input.drone_alt_msl);
        report.drone_altitude_rel = input.drone_alt_rel;
        report.drone_altitude_msl = input.drone_alt_msl;
        
        if (!report.altitude_safe) {
            report.add_error("Altitude limit exceeded");
        }
        
        // Target timeout
        report.target_timeout_sec = input.time_since_target;
        if (input.target_valid && input.time_since_target > input.timeout_threshold) {
            report.add_warning("Target GPS timeout");
        }
        
        // Emergency state
        report.emergency_active = emergency_->is_active();
        report.armed = input.armed;
        report.guided = input.guided;
        
        return report;
    }
    
private:
    GPSHealthMonitor* gps_monitor_;
    EmergencyController* emergency_;
};

} // namespace core
} // namespace drone_follow
