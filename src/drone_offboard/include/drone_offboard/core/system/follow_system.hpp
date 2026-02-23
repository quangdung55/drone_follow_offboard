#pragma once

#include "../estimator/target_estimator.hpp"
#include "../control/follow_controller.hpp"
#include "../control/yaw_controller.hpp"
#include "../safety/estimate_validator.hpp"
#include "../safety/velocity_limiter.hpp"
#include "../safety/emergency_controller.hpp"
#include "../supervisor/follow_supervisor.hpp"
#include <memory>

namespace drone_follow {
namespace core {

/**
 * @brief Unified Follow System - Single facade for all core algorithms
 * 
 * ROS node chỉ cần gọi 1-2 methods, không quan tâm internal components
 */
class FollowSystem {
public:
    struct Input {
        DroneState drone;
        TargetState target;
        double dt;
        double current_time_sec;
        
        // Raw state flags (NO policy interpretation!)
        bool armed;              // Raw armed flag from FCU
        bool guided;             // Raw guided mode flag from FCU
        bool origin_ready;       // Origin initialized
        double timeout_threshold; // Timeout parameter
    };
    
    struct Output {
        NavCommand cmd;              // ALWAYS valid NavCommand (even for emergency!)
        FollowSupervisor::State state;
        const char* message;
    };
    
    FollowSystem(
        std::unique_ptr<FollowController> follow_ctrl,
        std::unique_ptr<YawController> yaw_ctrl,
        std::unique_ptr<EstimateValidator> validator,
        std::unique_ptr<VelocityLimiter> limiter,
        std::unique_ptr<EmergencyController> emergency)
        : follow_controller_(std::move(follow_ctrl)),
          yaw_controller_(std::move(yaw_ctrl)),
          estimate_validator_(std::move(validator)),
          velocity_limiter_(std::move(limiter)),
          emergency_controller_(std::move(emergency)) {}
    
    /**
     * @brief Main update - Single call from ROS node
     * 
     * ROS chỉ cần: publish(output.cmd)
     * Không cần biết đó là emergency, stop, hay tracking!
     */
    Output update(const Input& input) {
        Output out;
        
        // Supervisor check policy (ALL policy decisions here!)
        FollowSupervisor::Input supervisor_in;
        supervisor_in.origin_ready = input.origin_ready;
        supervisor_in.drone_ready = input.drone.is_ready();
        supervisor_in.armed = input.armed;
        supervisor_in.guided = input.guided;
        supervisor_in.target_valid = input.target.valid;
        supervisor_in.time_since_target_sec = 
            input.current_time_sec - input.target.last_update_sec;
        supervisor_in.timeout_threshold_sec = input.timeout_threshold;
        supervisor_in.altitude_safe = 
            emergency_controller_->is_altitude_safe(input.drone.alt_rel, input.drone.alt_msl);
        
        auto supervisor_out = supervisor_.update(supervisor_in);
        
        out.state = supervisor_out.state;
        out.message = supervisor_out.status_message;
        
        // Emergency descent - Return NavCommand (NO special handling!)
        if (supervisor_out.should_emergency) {
            out.cmd.vel_forward = 0.0;
            out.cmd.vel_left = 0.0;
            out.cmd.vel_up = emergency_controller_->get_descent_command();
            out.cmd.yaw_rate = 0.0;
            return out;
        }
        
        // Stop command - Return NavCommand
        if (supervisor_out.should_stop) {
            out.cmd = {0, 0, 0, 0};
            return out;
        }
        
        // No action needed (disarmed/not guided) - Return zero command
        if (!supervisor_out.should_track) {
            out.cmd = {0, 0, 0, 0};
            return out;
        }
        
        // Normal tracking
        // Compute control
        NavCommand nav = follow_controller_->update(input.drone, input.target, input.dt);
        nav.yaw_rate = yaw_controller_->update(
            input.drone, input.target, input.dt, input.current_time_sec);
        
        // Apply safety limits (clamp in-place)
        velocity_limiter_->clamp(nav.vel_forward, nav.vel_left, 
                                 nav.vel_up, nav.yaw_rate);
        out.cmd = nav;
        
        return out;
    }
    
    // Accessors for components (for advanced use)
    YawController* yaw_controller() { return yaw_controller_.get(); }
    EmergencyController* emergency_controller() { return emergency_controller_.get(); }
    
private:
    std::unique_ptr<FollowController> follow_controller_;
    std::unique_ptr<YawController> yaw_controller_;
    std::unique_ptr<EstimateValidator> estimate_validator_;
    std::unique_ptr<VelocityLimiter> velocity_limiter_;
    std::unique_ptr<EmergencyController> emergency_controller_;
    FollowSupervisor supervisor_;
};

} // namespace core
} // namespace drone_follow
