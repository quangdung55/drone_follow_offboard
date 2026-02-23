/**
 * @file smart_follow_node_refactored_example.cpp
 * @brief Example of refactored ROS node using core architecture
 * 
 * BEFORE: 1500+ lines with all algorithms in callbacks
 * AFTER: ~400 lines - just ROS adapter layer
 * 
 * This demonstrates how to use the core components:
 * - TargetEstimator (GPS processing)
 * - FollowController (position control)
 * - YawController (yaw control)
 * - Safety validators
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>

// Core headers (NO ROS DEPENDENCY!)
#include "drone_offboard/core/types/drone_state.hpp"
#include "drone_offboard/core/types/target_state.hpp"
#include "drone_offboard/core/types/nav_command.hpp"
#include "drone_offboard/core/types/common.hpp"
#include "drone_offboard/core/types/parameters.hpp"
#include "drone_offboard/core/estimator/target_estimator.hpp"
#include "drone_offboard/core/control/follow_controller.hpp"
#include "drone_offboard/core/control/yaw_controller.hpp"
#include "drone_offboard/core/safety/estimate_validator.hpp"
#include "drone_offboard/core/safety/velocity_limiter.hpp"
#include "drone_offboard/core/safety/altitude_safety.hpp"

#include <mutex>
#include <chrono>

using namespace std::chrono_literals;
using namespace drone_follow::core;

/**
 * @brief Refactored Smart Follow Node - ROS Adapter Layer Only
 * 
 * This node is just a thin adapter that:
 * 1. Subscribes to ROS topics
 * 2. Converts ROS messages to core types
 * 3. Calls core algorithms
 * 4. Publishes results back to ROS
 */
class SmartFollowNodeRefactored : public rclcpp::Node {
public:
    SmartFollowNodeRefactored() : Node("smart_follow_node_refactored") {
        RCLCPP_INFO(get_logger(), "🚀 Starting Refactored Smart Follow Node");
        
        // =====================================================================
        // STEP 1: Load ROS parameters → Core parameter structs
        // =====================================================================
        EstimatorParams est_params = load_estimator_params();
        FollowControlParams ctrl_params = load_follow_control_params();
        YawControlParams yaw_params = load_yaw_control_params();
        
        // =====================================================================
        // STEP 2: Initialize CORE components (NO ROS!)
        // =====================================================================
        origin_manager_ = std::make_unique<OriginManager>();
        target_estimator_ = std::make_unique<TargetEstimator>(est_params, *origin_manager_);
        follow_controller_ = std::make_unique<FollowController>(ctrl_params);
        yaw_controller_ = std::make_unique<YawController>(yaw_params);
        estimate_validator_ = std::make_unique<EstimateValidator>(est_params);
        velocity_limiter_ = std::make_unique<VelocityLimiter>();
        altitude_safety_ = std::make_unique<AltitudeSafety>();
        
        // =====================================================================
        // STEP 3: Setup ROS pub/sub (ONLY ROS CODE IN THIS CLASS!)
        // =====================================================================
        setup_ros_interface();
        
        RCLCPP_INFO(get_logger(), "✅ Refactored node ready - Core architecture active");
        RCLCPP_INFO(get_logger(), "📊 Code reduction: ~73%% (1500 → 400 lines)");
    }
    
private:
    // =========================================================================
    // CORE COMPONENTS (Pure C++, NO ROS!)
    // =========================================================================
    std::unique_ptr<OriginManager> origin_manager_;
    std::unique_ptr<TargetEstimator> target_estimator_;
    std::unique_ptr<FollowController> follow_controller_;
    std::unique_ptr<YawController> yaw_controller_;
    std::unique_ptr<EstimateValidator> estimate_validator_;
    std::unique_ptr<VelocityLimiter> velocity_limiter_;
    std::unique_ptr<AltitudeSafety> altitude_safety_;
    
    // =========================================================================
    // STATE (Protected by mutex)
    // =========================================================================
    std::mutex state_mutex_;
    DroneState drone_state_;
    TargetState target_state_;
    
    // =========================================================================
    // ROS INTERFACE (Publishers, Subscribers, Timer)
    // =========================================================================
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr sub_origin_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_drone_gps_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_target_gps_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_gimbal_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // =========================================================================
    // PARAMETER LOADING (ROS → Core structs)
    // =========================================================================
    EstimatorParams load_estimator_params() {
        EstimatorParams params;
        this->declare_parameter("filter_enable", true);
        this->declare_parameter("filter_alpha", 0.7);
        this->declare_parameter("filter_beta", 0.1);
        this->declare_parameter("timeout", 3.0);
        
        params.filter_enable = this->get_parameter("filter_enable").as_bool();
        params.filter_alpha = this->get_parameter("filter_alpha").as_double();
        params.filter_beta = this->get_parameter("filter_beta").as_double();
        params.timeout_sec = this->get_parameter("timeout").as_double();
        // ... load other params
        
        return params;
    }
    
    FollowControlParams load_follow_control_params() {
        FollowControlParams params;
        this->declare_parameter("kp_pos", 0.1);
        this->declare_parameter("follow_height", 3.0);
        this->declare_parameter("accel_max_ne", 2.5);
        this->declare_parameter("jerk_max_ne", 5.0);
        
        params.kp_pos = this->get_parameter("kp_pos").as_double();
        params.follow_height = this->get_parameter("follow_height").as_double();
        params.accel_max_ne = this->get_parameter("accel_max_ne").as_double();
        params.jerk_max_ne = this->get_parameter("jerk_max_ne").as_double();
        // ... load other params
        
        return params;
    }
    
    YawControlParams load_yaw_control_params() {
        YawControlParams params;
        this->declare_parameter("kp_yaw", 0.05);
        this->declare_parameter("gimbal_deadzone", 5.0);
        
        params.kp_yaw = this->get_parameter("kp_yaw").as_double();
        params.gimbal_deadzone = this->get_parameter("gimbal_deadzone").as_double();
        // ... load other params
        
        return params;
    }
    
    // =========================================================================
    // ROS INTERFACE SETUP
    // =========================================================================
    void setup_ros_interface() {
        auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
        
        sub_origin_ = create_subscription<geographic_msgs::msg::GeoPointStamped>(
            "/ap/gps_global_origin/filtered", qos,
            std::bind(&SmartFollowNodeRefactored::cb_origin, this, std::placeholders::_1));
        
        sub_target_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/target/gps", qos,
            std::bind(&SmartFollowNodeRefactored::cb_target_gps, this, std::placeholders::_1));
        
        sub_gimbal_ = create_subscription<geometry_msgs::msg::Point>(
            "/gimbal/angle_error", 10,
            std::bind(&SmartFollowNodeRefactored::cb_gimbal, this, std::placeholders::_1));
        
        pub_cmd_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>("/ap/cmd_vel", 10);
        
        timer_ = create_wall_timer(20ms, 
            std::bind(&SmartFollowNodeRefactored::control_loop, this));
    }
    
    // =========================================================================
    // CALLBACKS (Just ROS → Core conversion, ~10-20 lines each)
    // =========================================================================
    
    /**
     * @brief Origin callback - Set EKF origin
     * BEFORE: 15 lines | AFTER: 5 lines
     */
    void cb_origin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {
        if (origin_manager_->set_origin(msg->position.latitude,
                                       msg->position.longitude,
                                       msg->position.altitude)) {
            RCLCPP_INFO(get_logger(), "EKF origin set: %.7f, %.7f",
                       msg->position.latitude, msg->position.longitude);
        }
    }
    
    /**
     * @brief Target GPS callback
     * BEFORE: ~200 lines of processing (jitter, filter, velocity est, etc.)
     * AFTER: ~15 lines (just conversion + core call)
     */
    void cb_target_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // Step 1: Convert ROS msg → Core measurement
        GPSMeasurement meas;
        meas.lat = msg->latitude;
        meas.lon = msg->longitude;
        meas.alt = msg->altitude;
        meas.timestamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
        meas.horizontal_accuracy = std::sqrt(msg->position_covariance[0]);
        
        // Step 2: Call CORE estimator (all logic here!)
        double local_time = this->now().seconds();
        if (!target_estimator_->update(meas, local_time)) {
            return;  // Estimator rejected update (invalid dt, timeout, etc.)
        }
        
        // Step 3: Update shared state (thread-safe)
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Copy core state to ROS-managed state
        const TargetState& core_state = target_estimator_->state();
        target_state_.lat = core_state.lat;
        target_state_.lon = core_state.lon;
        target_state_.alt = core_state.alt;
        target_state_.pos_enu = core_state.pos_enu;
        target_state_.vel_enu = core_state.vel_enu;
        target_state_.accel_enu = core_state.accel_enu;
        target_state_.heading_rad = core_state.heading_rad;
        target_state_.yaw_rate = core_state.yaw_rate;
        target_state_.valid = core_state.valid;
        target_state_.last_update_sec = core_state.last_update_sec;
        
        // That's it! No more algorithm code in callback.
    }
    
    /**
     * @brief Gimbal callback
     * BEFORE: 10 lines | AFTER: 5 lines
     */
    void cb_gimbal(const geometry_msgs::msg::Point::SharedPtr msg) {
        GimbalMeasurement meas;
        meas.pan_error_deg = msg->x;
        meas.timestamp_sec = this->now().seconds();
        
        yaw_controller_->update_gimbal(meas);
    }
    
    // =========================================================================
    // CONTROL LOOP (Main logic - now ~40 lines instead of 150+)
    // =========================================================================
    void control_loop() {
        // =====================================================================
        // Phase 1: Snapshot state (minimize lock time)
        // =====================================================================
        DroneState drone;
        TargetState target;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            drone = drone_state_;
            target = target_state_;
        }
        
        // =====================================================================
        // Phase 2: Safety checks (using core validators!)
        // =====================================================================
        if (!altitude_safety_->is_safe(drone)) {
            publish_cmd(altitude_safety_->get_emergency_descent_command());
            return;
        }
        
        if (!drone.is_ready() || !target.valid) {
            publish_stop();
            return;
        }
        
        // Check estimate error
        if (estimate_validator_->is_error_too_large(
                target,
                target_estimator_->last_estimate_pos(),
                target_estimator_->last_estimate_vel())) {
            RCLCPP_WARN(get_logger(), "Estimate error too large - resetting");
            target_estimator_->reset();
            return;
        }
        
        // =====================================================================
        // Phase 3: Calculate control (ALL LOGIC IN CORE!)
        // =====================================================================
        double dt = 0.02;  // 50Hz
        
        // Calculate navigation command (position + velocity control)
        NavCommand cmd = follow_controller_->update(drone, target, dt);
        
        // Calculate yaw rate (gimbal + heading follow)
        cmd.yaw_rate = yaw_controller_->update(drone, target, dt, this->now().seconds());
        
        // =====================================================================
        // Phase 4: Safety clamp and publish
        // =====================================================================
        auto check = velocity_limiter_->check(cmd.vel_forward, cmd.vel_left, 
                                             cmd.vel_up, cmd.yaw_rate);
        if (!check.passed) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Velocity exceeded: %s", check.reason.c_str());
        }
        
        velocity_limiter_->clamp(cmd.vel_forward, cmd.vel_left, cmd.vel_up, cmd.yaw_rate);
        publish_cmd(cmd);
    }
    
    // =========================================================================
    // HELPER FUNCTIONS (ROS publishing)
    // =========================================================================
    void publish_cmd(const NavCommand& cmd) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.twist.linear.x = cmd.vel_forward;
        msg.twist.linear.y = -cmd.vel_left;  // ROS convention
        msg.twist.linear.z = cmd.vel_up;
        msg.twist.angular.z = cmd.yaw_rate;
        
        pub_cmd_vel_->publish(msg);
    }
    
    void publish_stop() {
        NavCommand stop;
        stop.stop();
        publish_cmd(stop);
    }
};

// =============================================================================
// MAIN
// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartFollowNodeRefactored>());
    rclcpp::shutdown();
    return 0;
}
