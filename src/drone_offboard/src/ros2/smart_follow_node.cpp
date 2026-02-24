/**
 * @file smart_follow_node.cpp
 * @brief Smart Follow Node - Thin ROS2 Adapter Layer
 * 
 * ARCHITECTURE:
 * ===============
 * ROS2 Node: Chỉ lo pub/sub và quản lý tiến trình
 * Core Library: Xử lý toàn bộ thuật toán (NO ROS dependency)
 * 
 * Responsibilities:
 * ===================
 * ROS Layer:
 *  - Subscribe topics (GPS, pose, gimbal, status)
 *  - Publish commands (velocity setpoint)
 *  - Parameter management
 *  - Lifecycle & timing
 * 
 * Core Layer (drone_follow::core):
 *  - TargetEstimator: GPS processing, filtering, velocity estimation
 *  - FollowController: Position control với sqrt controller, kinematic shaping
 *  - YawController: Gimbal tracking + heading follow
 *  - Safety validators: Estimate checking, velocity limiting, altitude safety
 */

#include "drone_offboard/ros2/smart_follow_node.hpp"

// ROS messages (moved from header)
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <ardupilot_msgs/msg/status.hpp>

// STL (implementation only)
#include <cmath>
#include <algorithm>
#include <stdexcept>

// Core unified system (NO ROS DEPENDENCY!)
#include "drone_offboard/core/system/follow_system.hpp"
#include "drone_offboard/core/system/system_health.hpp"
#include "drone_offboard/core/estimator/target_estimator.hpp"
#include "drone_offboard/core/control/follow_controller.hpp"
#include "drone_offboard/core/control/yaw_controller.hpp"
#include "drone_offboard/core/control/ap_control.hpp"
#include "drone_offboard/core/control/ap_follow.hpp"
#include "drone_offboard/core/safety/estimate_validator.hpp"
#include "drone_offboard/core/safety/velocity_limiter.hpp"
#include "drone_offboard/core/safety/altitude_safety.hpp"
#include "drone_offboard/core/safety/gps_health_monitor.hpp"
#include "drone_offboard/core/safety/emergency_controller.hpp"
#include "drone_offboard/core/math/conversions.hpp"
#include "drone_offboard/core/math/JitterCorrection.h"

using namespace std::chrono_literals;
using namespace drone_follow::core;

/**
 * @brief Constructor - Initialize node with all parameters and filters
 */
SmartFollowNode::SmartFollowNode() : Node("smart_follow_node") {
    // Load ROS parameters
    setup_parameters();
    
    // Initialize core algorithms (NO ROS dependency)
    initialize_core_components();
    
    // Setup QoS profiles for ArduPilot DDS
    setup_qos();
    
    // Setup publishers, subscribers, and timer
    setup_pub_sub();

    // Initialize timing
    last_control_time_ = this->now();

    RCLCPP_INFO(get_logger(), "Smart Follow Node Started [ArduPilot Native DDS]");
    RCLCPP_INFO(get_logger(), "Core Architecture: Algorithms ROS-independent, unit testable");
    RCLCPP_INFO(get_logger(), "Features: Sqrt Controller, Kinematic Shaping, Terrain Follow");
    
    // Log offset mode
    const char* offset_mode_str[] = {"NED", "RELATIVE", "VELOCITY"};
    int offset_idx = static_cast<int>(param_offset_type_);
    RCLCPP_INFO(get_logger(), "Offset Mode: %s (North=%.2f, East=%.2f, Down=%.2f)",
                offset_mode_str[offset_idx],
                param_offset_ned_.x(), param_offset_ned_.y(), param_offset_ned_.z());
}

/**
 * @brief Destructor - Must be defined in .cpp for unique_ptr of forward-declared types
 */
SmartFollowNode::~SmartFollowNode() = default;

// --- SETUP FUNCTIONS ---

void SmartFollowNode::setup_parameters() {
    // Basic Follow Params
    this->declare_parameter("follow_dist", 5.0);
    this->declare_parameter("follow_height", 3.0);
    this->declare_parameter("prediction_time", 0.5);
    this->declare_parameter("timeout", 3.0);

    // PID & FeedForward Gains
    this->declare_parameter("kp_pos", 0.1);
    this->declare_parameter("kp_vel_z", 1.5);
    this->declare_parameter("kp_yaw", 0.05);
    this->declare_parameter("ff_lateral_gain", 0.85);
    this->declare_parameter("ff_centripetal_gain", 2.0);

    // Constraints
    this->declare_parameter("accel_max_ne", 2.5);
    this->declare_parameter("jerk_max_ne", 5.0);
    this->declare_parameter("accel_max_d", 1.0);      // Reduced from 2.5 to 1.0
    this->declare_parameter("jerk_max_d", 2.0);       // Reduced from 5.0 to 2.0
    this->declare_parameter("accel_max_h", 90.0);
    this->declare_parameter("jerk_max_h", 360.0);
    this->declare_parameter("gimbal_deadzone", 5.0);

    // Alpha-Beta Filter Params
    this->declare_parameter("filter_enable", true);
    this->declare_parameter("filter_alpha", 0.7);
    this->declare_parameter("filter_beta", 0.1);

    // Advanced Features
    this->declare_parameter("adaptive_distance_enable", false);
    this->declare_parameter("follow_dist_min", 3.0);
    this->declare_parameter("follow_dist_max", 15.0);
    this->declare_parameter("distance_speed_gain", 1.5);
    this->declare_parameter("heading_blend_enable", false);
    this->declare_parameter("heading_blend_weight", 0.3);
    this->declare_parameter("heading_follow_kp", 0.5);
    this->declare_parameter("terrain_follow_enable", true);
    this->declare_parameter("offset_type", 2);
    this->declare_parameter("offset_north", 0.0);
    this->declare_parameter("offset_east", 0.0);
    this->declare_parameter("offset_down", 0.0);
    this->declare_parameter("jitter_correction_enable", true);
    this->declare_parameter("jitter_max_lag_ms", 500);
    this->declare_parameter("jitter_convergence_loops", 100);
    this->declare_parameter("filter_beta_fast", 0.1);
    this->declare_parameter("filter_beta_slow", 0.05);
    this->declare_parameter("filter_residual_threshold", 2.0);

    // Cache parameters
    param_follow_dist_ = this->get_parameter("follow_dist").as_double();
    param_follow_height_ = this->get_parameter("follow_height").as_double();
    param_pred_time_ = this->get_parameter("prediction_time").as_double();
    param_timeout_ = this->get_parameter("timeout").as_double();
    param_kp_pos_ = this->get_parameter("kp_pos").as_double();
    param_kp_vel_z_ = this->get_parameter("kp_vel_z").as_double();
    param_kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    param_ff_lateral_gain_ = this->get_parameter("ff_lateral_gain").as_double();
    param_ff_centripetal_gain_ = this->get_parameter("ff_centripetal_gain").as_double();
    
    param_accel_max_ne_ = this->get_parameter("accel_max_ne").as_double();
    param_jerk_max_ne_ = this->get_parameter("jerk_max_ne").as_double();
    param_accel_max_d_ = this->get_parameter("accel_max_d").as_double();
    param_jerk_max_d_ = this->get_parameter("jerk_max_d").as_double();
    param_accel_max_h_ = this->get_parameter("accel_max_h").as_double() * ::DEG_TO_RAD;
    param_jerk_max_h_ = this->get_parameter("jerk_max_h").as_double() * ::DEG_TO_RAD;
    
    param_deadzone_ = this->get_parameter("gimbal_deadzone").as_double();
    param_filter_enable_ = this->get_parameter("filter_enable").as_bool();
    param_filter_alpha_ = this->get_parameter("filter_alpha").as_double();
    param_filter_beta_ = this->get_parameter("filter_beta").as_double();
    param_filter_beta_fast_ = this->get_parameter("filter_beta_fast").as_double();
    param_filter_beta_slow_ = this->get_parameter("filter_beta_slow").as_double();
    param_filter_residual_threshold_ = this->get_parameter("filter_residual_threshold").as_double();
    
    param_adaptive_dist_enable_ = this->get_parameter("adaptive_distance_enable").as_bool();
    param_follow_dist_min_ = this->get_parameter("follow_dist_min").as_double();
    param_follow_dist_max_ = this->get_parameter("follow_dist_max").as_double();
    param_dist_speed_gain_ = this->get_parameter("distance_speed_gain").as_double();
    
    param_heading_blend_enable_ = this->get_parameter("heading_blend_enable").as_bool();
    param_heading_blend_weight_ = this->get_parameter("heading_blend_weight").as_double();
    param_heading_follow_kp_ = this->get_parameter("heading_follow_kp").as_double();
    
    param_terrain_follow_enable_ = this->get_parameter("terrain_follow_enable").as_bool();
    
    int offset_type_int = this->get_parameter("offset_type").as_int();
    param_offset_type_ = static_cast<drone_follow::core::OffsetType>(offset_type_int);
    param_offset_ned_.x() = this->get_parameter("offset_north").as_double();
    param_offset_ned_.y() = this->get_parameter("offset_east").as_double();
    param_offset_ned_.z() = this->get_parameter("offset_down").as_double();
    
    param_jitter_correction_enable_ = this->get_parameter("jitter_correction_enable").as_bool();
    param_jitter_max_lag_ms_ = this->get_parameter("jitter_max_lag_ms").as_int();
    param_jitter_convergence_loops_ = this->get_parameter("jitter_convergence_loops").as_int();
}

/**
 * @brief Initialize Core Components (NO ROS DEPENDENCY!)
 * 
 * Tạo unified FollowSystem facade - ROS chỉ gọi 1 method
 */
void SmartFollowNode::initialize_core_components() {
    // Target Estimator
    EstimatorParams est_params;
    est_params.filter_alpha = param_filter_alpha_;
    est_params.filter_beta_slow = param_filter_beta_slow_;
    est_params.filter_beta_fast = param_filter_beta_fast_;
    est_params.filter_residual_threshold = param_filter_residual_threshold_;
    est_params.jitter_correction_enable = param_jitter_correction_enable_;
    est_params.jitter_max_lag_ms = param_jitter_max_lag_ms_;
    est_params.jitter_convergence_loops = param_jitter_convergence_loops_;
    
    target_estimator_ = std::make_unique<TargetEstimator>(est_params, origin_manager_);
    
    // Follow Controller
    FollowControlParams follow_params;
    follow_params.kp_pos = param_kp_pos_;
    follow_params.accel_max_ne = param_accel_max_ne_;
    follow_params.jerk_max_ne = param_jerk_max_ne_;
    follow_params.accel_max_d = param_accel_max_d_;
    follow_params.jerk_max_d = param_jerk_max_d_;
    follow_params.follow_height = param_follow_height_;
    follow_params.follow_dist = param_follow_dist_;
    follow_params.adaptive_distance_enable = param_adaptive_dist_enable_;
    follow_params.follow_dist_min = param_follow_dist_min_;
    follow_params.follow_dist_max = param_follow_dist_max_;
    follow_params.distance_speed_gain = param_dist_speed_gain_;
    follow_params.terrain_follow_enable = param_terrain_follow_enable_;
    follow_params.offset_type = param_offset_type_;
    follow_params.offset_ned = param_offset_ned_;
    
    auto follow_controller = std::make_unique<FollowController>(follow_params);
    
    // Yaw Controller
    YawControlParams yaw_params;
    yaw_params.kp_yaw = param_kp_yaw_;
    yaw_params.heading_follow_kp = param_heading_follow_kp_;
    yaw_params.gimbal_deadzone = param_deadzone_;
    yaw_params.heading_blend_weight = param_heading_blend_weight_;
    yaw_params.heading_blend_enable = param_heading_blend_enable_;
    yaw_params.accel_max_h = param_accel_max_h_;
    yaw_params.jerk_max_h = param_jerk_max_h_;
    
    auto yaw_controller = std::make_unique<YawController>(yaw_params);
    
    // Safety Components
    auto estimate_validator = std::make_unique<EstimateValidator>(est_params);
    auto velocity_limiter = std::make_unique<VelocityLimiter>();
    auto emergency_controller = std::make_unique<EmergencyController>(
        ::MAX_ALTITUDE_AGL, ::MAX_ALTITUDE_MSL, ::CMD_VEL_VERTICAL_MAX);
    
    // GPS Health Monitor
    gps_health_monitor_ = std::make_unique<GPSHealthMonitor>(
        ::GPS_COVARIANCE_WARN_THRESHOLD,
        ::GPS_COVARIANCE_MAX_THRESHOLD,
        ::GPS_FROZEN_TIMEOUT);
    
    // Create unified FollowSystem facade
    follow_system_ = std::make_unique<FollowSystem>(
        std::move(follow_controller),
        std::move(yaw_controller),
        std::move(estimate_validator),
        std::move(velocity_limiter),
        std::move(emergency_controller)
    );
    
    // System Health Monitor
    system_health_monitor_ = std::make_unique<SystemHealthMonitor>(
        gps_health_monitor_.get(),
        follow_system_->emergency_controller()
    );
    
    RCLCPP_INFO(get_logger(), " Core unified system initialized successfully");
}

void SmartFollowNode::setup_qos() {
    qos_best_effort_ = rclcpp::SensorDataQoS();
    qos_best_effort_.keep_last(1);
    qos_best_effort_.best_effort();
    qos_best_effort_.durability_volatile();
}

void SmartFollowNode::setup_pub_sub() {
    sub_global_origin_ = create_subscription<geographic_msgs::msg::GeoPointStamped>(
        "/ap/gps_global_origin/filtered", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_global_origin, this, std::placeholders::_1));

    sub_drone_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ap/navsat", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_drone_gps, this, std::placeholders::_1));

    sub_drone_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ap/pose/filtered", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_drone_pose, this, std::placeholders::_1));

    sub_target_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target/gps", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_target_gps, this, std::placeholders::_1));

    sub_gimbal_ = create_subscription<geometry_msgs::msg::Point>(
        "/gimbal/angle_error", 10,
        std::bind(&SmartFollowNode::cb_gimbal_angle, this, std::placeholders::_1));

    sub_status_ = create_subscription<ardupilot_msgs::msg::Status>(
        "/ap/status", 10,
        std::bind(&SmartFollowNode::cb_status, this, std::placeholders::_1));

    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>("/ap/cmd_vel", 10);
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/smart_follow/diagnostics", 10);

    timer_ = create_wall_timer(20ms, std::bind(&SmartFollowNode::control_loop, this));
    diagnostics_timer_ = create_wall_timer(1000ms, 
        std::bind(&SmartFollowNode::publish_diagnostics, this));
}

// --- CALLBACKS ---

void SmartFollowNode::cb_global_origin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {
    if (origin_manager_.set_origin(
            msg->position.latitude,
            msg->position.longitude,
            msg->position.altitude)) {
        RCLCPP_INFO(get_logger(),
                    "EKF Origin Set: lat=%.7f, lon=%.7f, alt=%.2f",
                    msg->position.latitude, 
                    msg->position.longitude, 
                    msg->position.altitude);
    }
}

void SmartFollowNode::cb_drone_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Update GPS health using core monitor
    double cov_x = msg->position_covariance[0];
    double cov_y = msg->position_covariance[4];
    bool has_cov = (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
    
    GPSHealthData health_data;
    gps_health_monitor_->update_from_covariance(cov_x, cov_y, has_cov, health_data);
    
    // Check frozen
    double now_sec = this->now().seconds();
    if (gps_health_monitor_->is_frozen(now_sec, drone_gps_health_.last_update.seconds())) {
        drone_gps_health_.frozen_detected = true;
    }
    
    // Copy to ROS state
    drone_gps_health_.horizontal_accuracy = health_data.horizontal_accuracy;
    drone_gps_health_.covariance = health_data.covariance;
    drone_gps_health_.has_covariance = health_data.has_covariance;
    drone_gps_health_.last_update = this->now();
    
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "Drone GPS: No valid fix");
        drone_state_.gps_valid = false;
        return;
    }
    
    // Check health using core monitor
    if (gps_health_monitor_->needs_warning(health_data)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Drone GPS accuracy degraded: %.1fm (covariance: %.1f)",
                            health_data.horizontal_accuracy,
                            health_data.covariance);
    }
    
    if (!gps_health_monitor_->is_healthy(health_data)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Drone GPS unhealthy - rejecting update");
        drone_state_.gps_valid = false;
        return;
    }
    
    if (!origin_manager_.is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Drone GPS received but origin not ready");
        return;
    }
    
    drone_state_.lat = msg->latitude;
    drone_state_.lon = msg->longitude;
    drone_state_.alt_msl = msg->altitude;
    
    double east, north;
    if (origin_manager_.gps_to_enu(msg->latitude, msg->longitude, east, north)) {
        drone_state_.pos_enu.x() = east;
        drone_state_.pos_enu.y() = north;
        
        double origin_lat, origin_lon, origin_alt;
        origin_manager_.get_origin(origin_lat, origin_lon, origin_alt);
        drone_state_.pos_enu.z() = msg->altitude - origin_alt;
        
        drone_state_.gps_valid = true;
        drone_state_.last_gps_update = this->now();
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to convert drone GPS to ENU");
        drone_state_.gps_valid = false;
    }
}

void SmartFollowNode::cb_drone_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    drone_state_.alt_rel = msg->pose.position.z;

    // Use core conversion utility (NO algorithm logic in ROS layer)
    drone_state_.yaw = drone_follow::core::Conversions::quaternion_to_yaw(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    
    drone_state_.pose_valid = true;
    drone_state_.last_pose_update = this->now();
}

/**
 * @brief Callback for target GPS updates - performs filtering and velocity estimation

 * 
 * Processing pipeline:
 * 1. Validate GPS fix quality
 * 2. Apply jitter correction to timestamp (compensate for network delay)
 * 3. Convert GPS to Map-ENU coordinates
 * 4. Estimate velocity using finite differences
 * 5. Apply Alpha-Beta filter for noise reduction
 * 6. Calculate acceleration with low-pass filtering
 * 7. Update heading and yaw rate from velocity vector
 * 
 * @param msg Target GPS position (WGS84)
 */
void SmartFollowNode::cb_target_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // GPS health tracking using core monitor
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        double cov_x = msg->position_covariance[0];
        double cov_y = msg->position_covariance[4];
        bool has_cov = (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
        
        GPSHealthData health_data;
        gps_health_monitor_->update_from_covariance(cov_x, cov_y, has_cov, health_data);
        
        // Check frozen
        double now_sec = this->now().seconds();
        if (gps_health_monitor_->is_frozen(now_sec, target_gps_health_.last_update.seconds())) {
            target_gps_health_.frozen_detected = true;
        }
        
        // Copy to ROS state
        target_gps_health_.horizontal_accuracy = health_data.horizontal_accuracy;
        target_gps_health_.covariance = health_data.covariance;
        target_gps_health_.has_covariance = health_data.has_covariance;
        target_gps_health_.last_update = this->now();
    }
    
    // Safety: Reject poor GPS quality
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "Target GPS: No valid fix");
        return;
    }
    
    // Convert ROS message → Core GPSMeasurement
    GPSMeasurement meas;
    meas.lat = msg->latitude;
    meas.lon = msg->longitude;
    meas.alt = msg->altitude;
    meas.timestamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    meas.horizontal_accuracy = msg->position_covariance[0];
    meas.has_covariance = (msg->position_covariance_type != 
                           sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
    
    // Get local time
    double local_time_sec = this->now().seconds();
    
    // Core algorithm: Target estimator xử lý toàn bộ (filtering, velocity, acceleration)
    bool success = target_estimator_->update(meas, local_time_sec);
    
    if (!success) {
        return;
    }
    
    // Copy result back to ROS layer
    std::lock_guard<std::mutex> lock(state_mutex_);
    const drone_follow::core::TargetState& core_state = target_estimator_->state();
    
    target_state_.pos_enu = core_state.pos_enu;
    target_state_.vel_enu = core_state.vel_enu;
    target_state_.accel_enu = core_state.accel_enu;
    target_state_.heading_rad = core_state.heading_rad;
    target_state_.yaw_rate = core_state.yaw_rate;
    target_state_.lat = core_state.lat;
    target_state_.lon = core_state.lon;
    target_state_.alt = core_state.alt;
    target_state_.valid = core_state.valid;
    target_state_.last_update = this->now();
}

void SmartFollowNode::cb_gimbal_angle(const geometry_msgs::msg::Point::SharedPtr msg) {
    // Update gimbal tracker in yaw controller
    GimbalMeasurement gimbal;
    gimbal.pan_error_deg = msg->x;
    gimbal.timestamp_sec = this->now().seconds();
    follow_system_->yaw_controller()->update_gimbal(gimbal);
}

void SmartFollowNode::cb_status(const ardupilot_msgs::msg::Status::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    is_armed_ = msg->armed;
    current_mode_int_ = msg->mode;
    is_guided_ = (current_mode_int_ == 4);

    static bool last_armed = false;
    static int last_mode = -1;

    if (is_armed_ != last_armed || current_mode_int_ != last_mode) {
        // Load atomic values before passing to macro
        bool armed_val = is_armed_.load();
        int mode_val = current_mode_int_.load();
        bool guided_val = is_guided_.load();
        
        RCLCPP_INFO(get_logger(), "Status Update -> Armed: %s | Mode: %d (Guided: %s)", 
            armed_val ? "YES" : "NO", 
            mode_val,
            guided_val ? "YES" : "NO");
        last_armed = is_armed_;
        last_mode = current_mode_int_;
    }
}

// --- CONTROL LOOP ---

/**
 * @brief Main control loop (50Hz) - Pure adapter to core FollowSystem
 * 
 * ROS Layer: Snapshot state → Call core → Publish command
 * Core Layer: ALL decisions (policy, safety, control)
 * 
 * Node không biết:
 * - Command là emergency hay tracking
 * - Khi nào nên stop
 * - Armed/guided có nghĩa gì
 * 
 * Node CHỈ biết: publish(command)
 */
void SmartFollowNode::control_loop() {
    // 1. Snapshot state (minimal lock time)
    ::DroneState drone_snapshot_ros;
    ::TargetState target_snapshot_ros;
    bool armed, guided;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        drone_snapshot_ros = drone_state_;
        target_snapshot_ros = target_state_;
        armed = is_armed_;
        guided = is_guided_;
    }
    
    // 2. Calculate dt
    double now_sec = this->now().seconds();
    double dt = ::CONTROL_LOOP_DT_DEFAULT;
    
    if (last_control_time_.seconds() > 0.0) {
        double calculated_dt = now_sec - last_control_time_.seconds();
        if (calculated_dt > 0.001 && calculated_dt < 1.0) {
            dt = calculated_dt;
        }
    }
    last_control_time_ = this->now();
    
    // 3. Convert ROS types to core types
    drone_follow::core::DroneState drone_core;
    drone_core.lat = drone_snapshot_ros.lat;
    drone_core.lon = drone_snapshot_ros.lon;
    drone_core.alt_msl = drone_snapshot_ros.alt_msl;
    drone_core.alt_rel = drone_snapshot_ros.alt_rel;
    drone_core.pos_enu = drone_snapshot_ros.pos_enu;
    drone_core.yaw = drone_snapshot_ros.yaw;
    drone_core.gps_valid = drone_snapshot_ros.gps_valid;
    drone_core.pose_valid = drone_snapshot_ros.pose_valid;
    drone_core.last_gps_update_sec = drone_snapshot_ros.last_gps_update.seconds();
    drone_core.last_pose_update_sec = drone_snapshot_ros.last_pose_update.seconds();
    
    drone_follow::core::TargetState target_core;
    target_core.lat = target_snapshot_ros.lat;
    target_core.lon = target_snapshot_ros.lon;
    target_core.alt = target_snapshot_ros.alt;
    target_core.pos_enu = target_snapshot_ros.pos_enu;
    target_core.vel_enu = target_snapshot_ros.vel_enu;
    target_core.accel_enu = target_snapshot_ros.accel_enu;
    target_core.heading_rad = target_snapshot_ros.heading_rad;
    target_core.yaw_rate = target_snapshot_ros.yaw_rate;
    target_core.valid = target_snapshot_ros.valid;
    target_core.last_update_sec = target_snapshot_ros.last_update.seconds();
    
    // 4. Prepare input - Pass RAW state (NO policy interpretation!)
    FollowSystem::Input input;
    input.drone = drone_core;
    input.target = target_core;
    input.dt = dt;
    input.current_time_sec = now_sec;
    input.armed = armed;              // Raw flag only
    input.guided = guided;            // Raw flag only
    input.origin_ready = origin_manager_.is_ready();
    input.timeout_threshold = param_timeout_;
    
    // 5. Single call to core - ALL logic inside!
    auto output = follow_system_->update(input);
    
    // 6. Log state changes (informational only)
    static FollowSupervisor::State last_state = FollowSupervisor::State::IDLE;
    if (output.state != last_state) {
        RCLCPP_INFO(get_logger(), "State: %s", output.message);
        last_state = output.state;
    }
    
    // 7. Convert core NavCommand to ROS NavCommand and publish
    ::NavCommand ros_cmd;
    ros_cmd.vel_forward = output.cmd.vel_forward;
    ros_cmd.vel_left = output.cmd.vel_left;
    ros_cmd.vel_up = output.cmd.vel_up;
    ros_cmd.yaw_rate = output.cmd.yaw_rate;
    publish_cmd(ros_cmd);
    
    // 8. Log non-zero commands (debugging only)
    if (std::abs(ros_cmd.vel_forward) > 0.01 || 
        std::abs(ros_cmd.vel_left) > 0.01 || 
        std::abs(ros_cmd.vel_up) > 0.01 || 
        std::abs(ros_cmd.yaw_rate) > 0.01) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                            "CMD: fwd=%.2f left=%.2f up=%.2f yaw=%.2f",
                            ros_cmd.vel_forward, ros_cmd.vel_left, 
                            ros_cmd.vel_up, ros_cmd.yaw_rate);
    }
}

// ============================================================================
// COMMAND PUBLISHING
// ============================================================================

/**
 * @brief Publish velocity command to ArduPilot via DDS
 * @param cmd Navigation command in body frame
 */
void SmartFollowNode::publish_cmd(const ::NavCommand& cmd) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.twist.linear.x = cmd.vel_forward;
    msg.twist.linear.y = -cmd.vel_left;
    msg.twist.linear.z = cmd.vel_up;
    msg.twist.angular.z = cmd.yaw_rate;

    pub_cmd_vel_->publish(msg);
}

void SmartFollowNode::stop_drone() {
    ::NavCommand stop_cmd = {0, 0, 0, 0};
    publish_cmd(stop_cmd);
}

// ============================================================================
// DIAGNOSTICS - Thin wrapper to core SystemHealthMonitor
// ============================================================================

void SmartFollowNode::publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "smart_follow_node";
    diag.hardware_id = "ardupilot_drone";
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Convert ROS GPS health to core type
    drone_follow::core::GPSHealthData drone_gps_core;
    drone_gps_core.horizontal_accuracy = drone_gps_health_.horizontal_accuracy;
    drone_gps_core.covariance = drone_gps_health_.covariance;
    drone_gps_core.last_update_sec = drone_gps_health_.last_update.seconds();
    drone_gps_core.frozen_detected = drone_gps_health_.frozen_detected;
    drone_gps_core.has_covariance = drone_gps_health_.has_covariance;
    drone_gps_core.satellites = drone_gps_health_.satellites;
    
    drone_follow::core::GPSHealthData target_gps_core;
    target_gps_core.horizontal_accuracy = target_gps_health_.horizontal_accuracy;
    target_gps_core.covariance = target_gps_health_.covariance;
    target_gps_core.last_update_sec = target_gps_health_.last_update.seconds();
    target_gps_core.frozen_detected = target_gps_health_.frozen_detected;
    target_gps_core.has_covariance = target_gps_health_.has_covariance;
    target_gps_core.satellites = target_gps_health_.satellites;
    
    // Prepare input for core health monitor
    drone_follow::core::SystemHealthMonitor::Input health_input;
    health_input.origin_ready = origin_manager_.is_ready();
    health_input.drone_gps_valid = drone_state_.gps_valid;
    health_input.drone_gps_health = drone_gps_core;
    health_input.target_valid = target_state_.valid;
    health_input.target_gps_health = target_gps_core;
    health_input.drone_alt_rel = drone_state_.alt_rel;
    health_input.drone_alt_msl = drone_state_.alt_msl;
    health_input.time_since_target = (this->now() - target_state_.last_update).seconds();
    health_input.timeout_threshold = param_timeout_;
    health_input.armed = is_armed_.load();
    health_input.guided = is_guided_.load();
    
    // Core evaluates health
    auto report = system_health_monitor_->evaluate(health_input);
    
    // Convert core report to ROS DiagnosticStatus
    switch (report.level) {
        case SystemHealthReport::Level::OK:
            diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diag.message = "System healthy";
            break;
        case SystemHealthReport::Level::WARN:
            diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag.message = "System warnings active";
            break;
        case SystemHealthReport::Level::ERROR:
            diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diag.message = "System errors detected";
            break;
    }
    
    // Add detailed key-value pairs
    auto add_kv = [&diag](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        diag.values.push_back(kv);
    };
    
    add_kv("armed", report.armed ? "true" : "false");
    add_kv("guided", report.guided ? "true" : "false");
    add_kv("origin_ready", report.origin_ready ? "true" : "false");
    add_kv("drone_gps_valid", report.drone_gps_valid ? "true" : "false");
    add_kv("drone_gps_accuracy", std::to_string(report.drone_gps_accuracy) + "m");
    add_kv("drone_altitude_rel", std::to_string(report.drone_altitude_rel) + "m");
    add_kv("drone_altitude_msl", std::to_string(report.drone_altitude_msl) + "m");
    add_kv("target_valid", report.target_valid ? "true" : "false");
    add_kv("target_gps_accuracy", std::to_string(report.target_gps_accuracy) + "m");
    add_kv("target_timeout", std::to_string(report.target_timeout_sec) + "s");
    add_kv("filter_enabled", param_filter_enable_ ? "true" : "false");
    add_kv("terrain_follow", param_terrain_follow_enable_ ? "true" : "false");
    add_kv("emergency_descent", report.emergency_active ? "true" : "false");
    
    // Add errors and warnings from core
    for (size_t i = 0; i < report.errors.size(); ++i) {
        add_kv("error_" + std::to_string(i), report.errors[i]);
    }
    for (size_t i = 0; i < report.warnings.size(); ++i) {
        add_kv("warning_" + std::to_string(i), report.warnings[i]);
    }
    
    pub_diagnostics_->publish(diag);
}
