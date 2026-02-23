#pragma once

#include "../types/common.hpp"

namespace drone_follow {
namespace core {

/**
 * @brief Follow mission state machine (NO ROS dependency)
 * 
 * Quản lý policy logic: khi nào tracking, khi nào stop, khi nào emergency
 */
class FollowSupervisor {
public:
    enum class State {
        IDLE,           // Waiting for prerequisites (origin, GPS, etc)
        DISARMED,       // Drone not armed
        NOT_GUIDED,     // Not in guided mode
        TARGET_LOST,    // Target invalid or timeout
        EMERGENCY,      // Altitude safety violation
        TRACKING        // Normal tracking operation
    };
    
    struct Input {
        bool origin_ready;
        bool drone_ready;
        bool armed;
        bool guided;
        bool target_valid;
        double time_since_target_sec;
        double timeout_threshold_sec;
        bool altitude_safe;
    };
    
    struct Output {
        State state;
        bool should_stop;           // Send zero velocity command
        bool should_emergency;      // Trigger emergency descent
        bool should_track;          // Execute normal tracking
        const char* status_message;
    };
    
    Output update(const Input& input) {
        Output out;
        
        // Priority check: Emergency first
        if (!input.altitude_safe) {
            out.state = State::EMERGENCY;
            out.should_stop = false;
            out.should_emergency = true;
            out.should_track = false;
            out.status_message = "EMERGENCY: Altitude limit exceeded";
            return out;
        }
        
        // Check prerequisites
        if (!input.origin_ready) {
            out.state = State::IDLE;
            out.should_stop = true;
            out.should_emergency = false;
            out.should_track = false;
            out.status_message = "Waiting for EKF origin";
            return out;
        }
        
        if (!input.drone_ready) {
            out.state = State::IDLE;
            out.should_stop = true;
            out.should_emergency = false;
            out.should_track = false;
            out.status_message = "Drone not ready";
            return out;
        }
        
        if (!input.armed) {
            out.state = State::DISARMED;
            out.should_stop = false;  // Don't send commands when disarmed
            out.should_emergency = false;
            out.should_track = false;
            out.status_message = "Disarmed";
            return out;
        }
        
        if (!input.guided) {
            out.state = State::NOT_GUIDED;
            out.should_stop = false;  // Don't send commands in other modes
            out.should_emergency = false;
            out.should_track = false;
            out.status_message = "Not in guided mode";
            return out;
        }
        
        // Check target validity
        if (!input.target_valid) {
            out.state = State::TARGET_LOST;
            out.should_stop = true;
            out.should_emergency = false;
            out.should_track = false;
            out.status_message = "Target invalid";
            return out;
        }
        
        if (input.time_since_target_sec > input.timeout_threshold_sec) {
            out.state = State::TARGET_LOST;
            out.should_stop = true;
            out.should_emergency = false;
            out.should_track = false;
            out.status_message = "Target timeout";
            return out;
        }
        
        // All checks passed - normal tracking
        out.state = State::TRACKING;
        out.should_stop = false;
        out.should_emergency = false;
        out.should_track = true;
        out.status_message = "Tracking";
        return out;
    }
};

} // namespace core
} // namespace drone_follow
