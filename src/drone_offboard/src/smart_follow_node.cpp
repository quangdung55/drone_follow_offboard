/**
 * @file smart_follow_node.cpp
 * @brief Implementation of SmartFollowNode for ArduPilot GPS-based target following
 * 
 * This node implements intelligent drone following using:
 * - GPS-based position tracking with Map-ENU frame conversion
 * - ArduPilot-style sqrt controller for smooth position tracking
 * - Kinematic input shaping (jerk/acceleration limiting)
 * - Alpha-beta filtering for velocity estimation and noise reduction
 * - Terrain-following mode (relative altitude to target)
 * - Adaptive distance based on target speed
 * - Gimbal-assisted yaw control with heading follow blending
 * 
 * Thread Safety:
 * - All state access is protected by state_mutex_
 * - Control loop uses snapshot pattern to minimize lock time
 * - Origin manager uses atomic operations for lock-free checks
 */

#include "drone_offboard/smart_follow_node.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor - Initialize node with all parameters and filters
 */
SmartFollowNode::SmartFollowNode() : Node("smart_follow_node") {
    // Load and cache all ROS parameters
    setup_parameters();
    
    // Setup QoS profiles for ArduPilot DDS (Best Effort required)
    setup_qos();
    
    // Create publishers, subscribers, and timer
    setup_pub_sub();

    // Initialize Alpha-Beta Filters
    filter_vel_n_.reset(0.0);
    filter_vel_e_.reset(0.0);
    filter_alt_.reset(0.0);

    // Initialize last control time and target state
    last_control_time_ = this->now();
    target_state_.last_update = this->now();
    
    // Initialize ArduPilot Follow Estimator
    ap_follow::KinematicParams kinematic_params;
    kinematic_params.accel_max_ne_mss = param_accel_max_ne_;
    kinematic_params.jerk_max_ne_msss = param_jerk_max_ne_;
    kinematic_params.accel_max_d_mss = param_accel_max_d_;
    kinematic_params.jerk_max_d_msss = param_jerk_max_d_;
    kinematic_params.accel_max_h_radss = param_accel_max_h_;
    kinematic_params.jerk_max_h_radsss = param_jerk_max_h_;
    kinematic_params.timeout_sec = param_timeout_;
    kinematic_params.pos_p = param_kp_pos_;
    follow_estimator_.set_kinematic_params(kinematic_params);

    RCLCPP_INFO(get_logger(), "Smart Follow Node Started [ArduPilot Native DDS]");
    RCLCPP_INFO(get_logger(), "Features: AP_Control Sqrt Controller, Kinematic Shaping, Terrain Follow");
    
    // Log offset mode
    const char* offset_mode_str[] = {"NED", "RELATIVE", "VELOCITY"};
    int offset_idx = static_cast<int>(param_offset_type_);
    RCLCPP_INFO(get_logger(), "Offset Mode: %s (North=%.2f, East=%.2f, Down=%.2f)",
                offset_mode_str[offset_idx],
                param_offset_ned_.x(), param_offset_ned_.y(), param_offset_ned_.z());
    
    // Log jitter correction status
    RCLCPP_INFO(get_logger(), "Jitter Correction: %s (Max Lag=%dms, Convergence=%d samples)",
                param_jitter_correction_enable_ ? "ENABLED" : "DISABLED",
                param_jitter_max_lag_ms_, param_jitter_convergence_loops_);
    
    // Log adaptive beta status
    RCLCPP_INFO(get_logger(), "Adaptive Beta: Fast=%.3f (>%.2fm), Slow=%.3f (<=%.2fm)",
                param_filter_beta_fast_, param_filter_residual_threshold_,
                param_filter_beta_slow_, param_filter_residual_threshold_);
    
    // Log filter status
    RCLCPP_INFO(get_logger(), "Alpha-Beta Filter: %s",
                param_filter_enable_ ? "ENABLED" : "DISABLED (Using raw velocity)");
}

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
    param_accel_max_h_ = this->get_parameter("accel_max_h").as_double() * DEG_TO_RAD;
    param_jerk_max_h_ = this->get_parameter("jerk_max_h").as_double() * DEG_TO_RAD;
    
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
    param_offset_type_ = static_cast<OffsetType>(offset_type_int);
    param_offset_ned_.x() = this->get_parameter("offset_north").as_double();
    param_offset_ned_.y() = this->get_parameter("offset_east").as_double();
    param_offset_ned_.z() = this->get_parameter("offset_down").as_double();
    
    param_jitter_correction_enable_ = this->get_parameter("jitter_correction_enable").as_bool();
    param_jitter_max_lag_ms_ = this->get_parameter("jitter_max_lag_ms").as_int();
    param_jitter_convergence_loops_ = this->get_parameter("jitter_convergence_loops").as_int();
    
    jitter_correction_.emplace(param_jitter_max_lag_ms_, param_jitter_convergence_loops_);
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
    
    // Update GPS health monitoring
    update_gps_health(msg, drone_gps_health_);
    check_gps_frozen(drone_gps_health_, this->now());
    
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "Drone GPS: No valid fix");
        drone_state_.gps_valid = false;
        return;
    }
    
    // Warn on degraded GPS accuracy
    if (drone_gps_health_.needs_warning()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Drone GPS accuracy degraded: %.1fm (covariance: %.1f)",
                            drone_gps_health_.horizontal_accuracy,
                            drone_gps_health_.covariance);
    }
    
    // Reject GPS with unacceptable accuracy
    if (!drone_gps_health_.is_healthy()) {
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

    Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    drone_state_.yaw = std::atan2(siny_cosp, cosy_cosp);
    
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
    // Update GPS health monitoring
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        update_gps_health(msg, target_gps_health_);
        check_gps_frozen(target_gps_health_, this->now());
    }
    
    // Safety check: Reject poor GPS quality
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "Target GPS: No valid fix");
        return;
    }
    
    // Warn on degraded accuracy
    if (target_gps_health_.needs_warning()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Target GPS accuracy degraded: %.1fm",
                            target_gps_health_.horizontal_accuracy);
    }
    
    // Reject unhealthy GPS
    if (!target_gps_health_.is_healthy()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Target GPS unhealthy - rejecting update");
        return;
    }
    
    if (!origin_manager_.is_ready()) {
        return;
    }
    
    // === TIMESTAMP JITTER CORRECTION ===
    // GPS messages may have variable network delays. We correct timestamps
    // to get consistent dt for velocity estimation.
    uint64_t offboard_usec = msg->header.stamp.sec * 1000000ULL + 
                             msg->header.stamp.nanosec / 1000ULL;
    uint64_t local_usec = this->now().nanoseconds() / 1000ULL;
    
    uint64_t corrected_usec;
    
    // Check if offboard timestamp is valid (non-zero and reasonable)
    bool offboard_timestamp_valid = (offboard_usec > 0) && 
                                     (offboard_usec < local_usec + 1000000ULL); // Not in future by >1s
    
    if (param_jitter_correction_enable_ && offboard_timestamp_valid && 
        jitter_correction_.has_value()) {
        // Apply AP_Follow style timestamp correction
        corrected_usec = jitter_correction_->correct_offboard_timestamp_usec(
            offboard_usec, local_usec);
        
        // Validate corrected timestamp is reasonable
        if (corrected_usec == 0 || corrected_usec > local_usec + 1000000ULL) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Jitter correction produced invalid timestamp, using local time");
            corrected_usec = local_usec;
        }
    } else {
        // Use local timestamp if jitter correction disabled or offboard timestamp invalid
        if (!offboard_timestamp_valid) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Offboard timestamp invalid (0x%lx), using local time", offboard_usec);
        }
        corrected_usec = local_usec;
    }
    
    double now_sec = corrected_usec / 1000000.0;
    
    double east, north;
    if (!origin_manager_.gps_to_enu(msg->latitude, msg->longitude, east, north)) {
        RCLCPP_ERROR(get_logger(), "Failed to convert target GPS to ENU");
        return;
    }
    
    Eigen::Vector2d new_pos(east, north);
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    auto dt_result = target_dt_validator_.validate(now_sec);
    
    // Handle reset condition (timeout only)
    if (dt_result.should_reset) {
        double time_diff = now_sec - target_dt_validator_.get_last_valid_time();
        RCLCPP_WARN(get_logger(), 
                   "Target GPS: Timeout (%.1fs since last valid update), resetting filters",
                   time_diff);
        
        filter_vel_e_.reset(0.0);
        filter_vel_n_.reset(0.0);
        filter_alt_.reset(msg->altitude);
        target_state_.reset();
        target_dt_validator_.reset();
        return;
    }
    
    // Skip update if dt is invalid but doesn't require reset (e.g., duplicate timestamp, first update)
    if (!dt_result.valid) {
        // This is normal - silently skip
        return;
    }
    
    // Log first valid update for debugging
    if (!target_state_.valid) {
        RCLCPP_INFO(get_logger(), "Target GPS: First valid update received");
    }
    
    // === VELOCITY ESTIMATION (if we have previous valid state) ===
    if (target_state_.valid && dt_result.valid) {
        double dt = dt_result.dt;
        
        // Calculate raw velocity using finite differences: v = Δpos / Δt
        Eigen::Vector2d delta = new_pos - target_state_.pos_enu.head<2>();
        Eigen::Vector2d raw_vel = delta / dt;
        
        double raw_speed = raw_vel.norm();
        if (raw_speed > MAX_TARGET_VELOCITY) {
            RCLCPP_WARN(get_logger(), 
                       "Target velocity unreasonable: %.2f m/s, rejecting update",
                       raw_speed);
            return;
        }
        
        double old_vel_e = target_state_.vel_enu.x();
        double old_vel_n = target_state_.vel_enu.y();
        
        if (param_filter_enable_) {
            filter_vel_e_.update_adaptive(raw_vel.x(), dt, param_filter_alpha_,
                                         param_filter_beta_fast_, 
                                         param_filter_beta_slow_,
                                         param_filter_residual_threshold_);
            filter_vel_n_.update_adaptive(raw_vel.y(), dt, param_filter_alpha_,
                                         param_filter_beta_fast_,
                                         param_filter_beta_slow_,
                                         param_filter_residual_threshold_);
            
            target_state_.vel_enu.x() = filter_vel_e_.x;
            target_state_.vel_enu.y() = filter_vel_n_.x;
        } else {
            target_state_.vel_enu.x() = raw_vel.x();
            target_state_.vel_enu.y() = raw_vel.y();
        }
        
        // === ACCELERATION ESTIMATION ===
        // Calculate instantaneous acceleration: a = Δv / Δt
        double instant_accel_e = (target_state_.vel_enu.x() - old_vel_e) / dt;
        double instant_accel_n = (target_state_.vel_enu.y() - old_vel_n) / dt;
        
        // Apply low-pass filter to smooth noisy acceleration
        // IIR filter: y[n] = α*x[n] + β*y[n-1], where β = (1-α)
        target_state_.accel_enu.x() = ACCEL_FILTER_ALPHA * instant_accel_e + 
                                      ACCEL_FILTER_BETA * target_state_.accel_enu.x();
        target_state_.accel_enu.y() = ACCEL_FILTER_ALPHA * instant_accel_n + 
                                      ACCEL_FILTER_BETA * target_state_.accel_enu.y();
        
        // Clamp acceleration to reasonable limits (prevents filter windup on GPS glitches)
        double accel_mag = target_state_.accel_enu.head<2>().norm();
        if (accel_mag > ACCEL_MAGNITUDE_CLAMP) {
            target_state_.accel_enu.head<2>() *= (ACCEL_MAGNITUDE_CLAMP / accel_mag);
        }
        
        double origin_lat, origin_lon, origin_alt;
        origin_manager_.get_origin(origin_lat, origin_lon, origin_alt);
        
        if (param_filter_enable_) {
            filter_alt_.update(msg->altitude, dt, param_filter_alpha_, param_filter_beta_);
            target_state_.alt = filter_alt_.x;
            target_state_.pos_enu.z() = filter_alt_.x - origin_alt;
        } else {
            target_state_.alt = msg->altitude;
            target_state_.pos_enu.z() = msg->altitude - origin_alt;
        }
        
        // === HEADING CALCULATION (from velocity vector) ===
        double speed = target_state_.vel_enu.head<2>().norm();
        if (speed > SPEED_THRESHOLD_HEADING) {
            // Calculate heading from velocity vector (ENU frame: 0=East, π/2=North)
            double new_heading = std::atan2(target_state_.vel_enu.y(), 
                                           target_state_.vel_enu.x());
            
            // Calculate angular difference with proper wrapping [-π, π]
            double diff_heading = new_heading - target_state_.heading_rad;
            while (diff_heading > M_PI) diff_heading -= 2.0 * M_PI;
            while (diff_heading < -M_PI) diff_heading += 2.0 * M_PI;
            
            // Estimate yaw rate: ω = Δθ / Δt
            double instant_yaw_rate = diff_heading / dt;
            
            // Low-pass filter yaw rate for smooth tracking
            target_state_.yaw_rate = YAW_RATE_FILTER_ALPHA * instant_yaw_rate + 
                                    YAW_RATE_FILTER_BETA * target_state_.yaw_rate;
            
            target_state_.heading_rad = new_heading;
        } else {
            // Target moving too slowly - heading unreliable
            target_state_.yaw_rate = 0.0;
        }
        
    } else if (!target_state_.valid) {
        filter_vel_e_.reset(0.0);
        filter_vel_n_.reset(0.0);
        filter_alt_.reset(msg->altitude);
        
        double origin_lat, origin_lon, origin_alt;
        origin_manager_.get_origin(origin_lat, origin_lon, origin_alt);
        target_state_.pos_enu.z() = msg->altitude - origin_alt;
        target_state_.alt = msg->altitude;
    }
    
    target_state_.pos_enu.x() = new_pos.x();
    target_state_.pos_enu.y() = new_pos.y();
    target_state_.lat = msg->latitude;
    target_state_.lon = msg->longitude;
    target_state_.last_update = rclcpp::Time(corrected_usec * 1000, 
                                            this->get_clock()->get_clock_type());
    target_state_.valid = true;
}

void SmartFollowNode::cb_gimbal_angle(const geometry_msgs::msg::Point::SharedPtr msg) {
    gimbal_tracker_.update(msg->x, this->now());
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
 * @brief Main control loop (50Hz) - calculates and publishes velocity commands
 * 
 * Phase 1: Snapshot state (minimal lock time)
 * Phase 2: Safety checks (origin, drone ready, mode, arm status)
 * Phase 3: Target validity and timeout checks
 * Phase 4: Validate dt (time delta)
 * Phase 5: Check estimate error (AP_Follow style)
 * Phase 6: Calculate kinematics (position + velocity control)
 * Phase 7: Calculate yaw rate (gimbal tracking + heading follow)
 * Phase 8: Sanity check and publish command
 */
void SmartFollowNode::control_loop() {
    // ========================================================================
    // PHASE 1: SNAPSHOT STATE (minimize mutex lock duration)
    // ========================================================================
    DroneState drone_snapshot;
    TargetState target_snapshot;
    bool armed, guided;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        drone_snapshot = drone_state_;
        target_snapshot = target_state_;
        armed = is_armed_;
        guided = is_guided_;
    }
    // Lock released - callbacks can now update state concurrently
    
    if (!origin_manager_.is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Waiting for EKF origin...");
        return;
    }
    
    if (!drone_snapshot.is_ready()) {
        stop_drone();
        return;
    }
    
    // ========================================================================
    // ALTITUDE SAFETY CHECK
    // ========================================================================
    if (!check_altitude_safety(drone_snapshot)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                             "ALTITUDE LIMIT EXCEEDED (AGL:%.1fm/%.1fm MSL:%.1fm/%.1fm) - Emergency descent",
                             drone_snapshot.alt_rel, MAX_ALTITUDE_AGL,
                             drone_snapshot.alt_msl, MAX_ALTITUDE_MSL);
        emergency_descent();
        return;
    } else {
        // Altitude is safe - reset emergency descent flag
        if (emergency_descent_active_.load()) {
            emergency_descent_active_.store(false);
            RCLCPP_INFO(get_logger(), "Altitude returned to safe level - resuming normal operation");
        }
    }
    
    if (!guided || !armed) {
        reset_shaping_state();
        return;
    }
    
    if (!target_snapshot.valid) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Target not valid - stopping drone");
        stop_drone();
        return;
    }
    
    double time_since_target = (this->now() - target_snapshot.last_update).seconds();
    if (time_since_target > param_timeout_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Target timeout: %.1fs > %.1fs",
                            time_since_target, param_timeout_);
        stop_drone();
        return;
    }
    
    double now_sec = this->now().seconds();
    auto dt_result = control_dt_validator_.validate(now_sec);
    
    if (dt_result.should_reset) {
        RCLCPP_WARN(get_logger(), 
                   "Control loop: Invalid dt detected, resetting shaping state");
        reset_shaping_state();
        return;
    }
    
    if (!dt_result.valid) {
        RCLCPP_DEBUG(get_logger(), "Control loop: Using default dt");
    }
    
    double dt = dt_result.dt;
    
    if (estimate_error_too_large(target_snapshot)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Estimate error too large, resetting filters");
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        filter_vel_e_.reset(0.0);
        filter_vel_n_.reset(0.0);
        return;
    }
    
    NavCommand nav = calculate_kinematics(drone_snapshot, target_snapshot, dt);
    nav.yaw_rate = calculate_yaw_rate(drone_snapshot, target_snapshot, dt);
    
    // Check if clamping is needed (log warning but don't stop)
    auto sanity_check = velocity_checker_.check(
        nav.vel_forward, nav.vel_left, nav.vel_up, nav.yaw_rate);
    
    if (!sanity_check.passed) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Velocity exceeded limits (%s), clamping to safe values", 
                            sanity_check.reason.c_str());
    }
    
    // Always clamp to safety limits
    velocity_checker_.clamp(nav.vel_forward, nav.vel_left, nav.vel_up, nav.yaw_rate);
    
    // Log non-zero commands
    if (std::abs(nav.vel_forward) > 0.01 || std::abs(nav.vel_left) > 0.01 || 
        std::abs(nav.vel_up) > 0.01 || std::abs(nav.yaw_rate) > 0.01) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                            "CMD: fwd=%.2f left=%.2f up=%.2f yaw=%.2f",
                            nav.vel_forward, nav.vel_left, nav.vel_up, nav.yaw_rate);
    } else {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                             "CMD: All zeros (on target or no movement needed)");
    }
    
    publish_cmd(nav);
    last_control_time_ = this->now();
}

/**
 * @brief Calculate navigation commands using ArduPilot-style kinematic control
 * 
 * Algorithm:
 * 1. Predict target future position using kinematic equation: P = P0 + V*t + 0.5*A*t²
 * 2. Calculate adaptive follow distance based on target speed (optional)
 * 3. Calculate offset in ENU frame (velocity-based, NED, or relative)
 * 4. Compute position error vector in Map-ENU frame
 * 5. Apply sqrt controller for horizontal position (X-Y)
 * 6. Apply kinematic shaping with jerk/acceleration limits
 * 7. Apply sqrt controller for vertical position (Z)
 * 8. Transform from ENU to drone body frame
 * 
 * @param drone Current drone state (snapshot)
 * @param target Current target state (snapshot)
 * @param dt Time delta since last control update
 * @return Navigation command in body frame (forward, left, up, yaw_rate)
 */

NavCommand SmartFollowNode::calculate_kinematics(
    const DroneState& drone,
    const TargetState& target,
    double dt)
{
    NavCommand cmd{};

    // =========================================================================
    // 0. DT SANITY (AP style)
    // =========================================================================
    if (dt < 0.002 || dt > 0.1) {
        dt = 0.02; // fallback 50Hz
    }

    // =========================================================================
    // 1. POSITION ERROR (ENU)
    // =========================================================================
    Eigen::Vector2d pos_error_xy =
        target.pos_enu.head<2>() - drone.pos_enu.head<2>();

    double err_mag = pos_error_xy.norm();

    // =========================================================================
    // 2. RESET SAFEGUARD (target jump / re-acquire)
    // =========================================================================
    const double RESET_RADIUS = 50.0;  // meters
    if (err_mag > RESET_RADIUS) {
        shaped_vel_xy_.setZero();
        shaped_accel_xy_.setZero();
    }

    // =========================================================================
    // 3. DESIRED VELOCITY FROM POSITION (AP sqrt controller)
    // =========================================================================
    // AP_Follow uses SCALAR sqrt_controller on magnitude, then multiply by direction
    Eigen::Vector2d vel_pos_target{0.0, 0.0};
    
    if (err_mag > 1e-3) {
        double vel_mag = ap_control::sqrt_controller(
            err_mag,                  // Position error magnitude
            param_kp_pos_,           // Position P gain (0.1)
            CMD_VEL_FORWARD_MAX,     // VELOCITY limit (not accel!)
            dt);
        
        vel_pos_target = pos_error_xy.normalized() * vel_mag;
    }

    // =========================================================================
    // 4. TARGET VELOCITY FEEDFORWARD (AP_Follow logic)
    // =========================================================================
    Eigen::Vector2d vel_ff{0.0, 0.0};

    double target_speed = target.vel_enu.head<2>().norm();
    double ff_scale = 0.0;

    if (target_speed > 0.2 && err_mag > 0.5) {

        double alignment =
            pos_error_xy.dot(target.vel_enu.head<2>()) /
            (err_mag * target_speed);

        ff_scale = std::clamp(alignment, 0.0, 1.0);

        // giảm FF khi target đang phanh
        if (target.vel_enu.head<2>().dot(target.accel_enu.head<2>()) < 0.0) {
            ff_scale *= 0.5;
        }

        vel_ff = target.vel_enu.head<2>() * ff_scale;
    }

    // =========================================================================
    // 5. TOTAL VELOCITY TARGET
    // =========================================================================
    Eigen::Vector2d vel_target_xy = vel_pos_target + vel_ff;

    // Clamp to safety limit
    const double VEL_MAX = CMD_VEL_FORWARD_MAX;
    if (vel_target_xy.norm() > VEL_MAX) {
        vel_target_xy = vel_target_xy.normalized() * VEL_MAX;
    }

    // =========================================================================
    // 6. ACCELERATION FEEDFORWARD (AP_Follow does NOT use this)
    // =========================================================================
    Eigen::Vector2d accel_ff{0.0, 0.0};

    // =========================================================================
    // 7. SHAPE VELOCITY → ACCELERATION (AP shape_vel_accel_xy)
    // =========================================================================
    ap_control::shape_vel_accel_xy(
        vel_target_xy,
        accel_ff,
        shaped_vel_xy_,         // STATE velocity
        shaped_accel_xy_,       // STATE acceleration
        param_accel_max_ne_,
        param_jerk_max_ne_,
        dt,
        true                    // limit total accel
    );

    // =========================================================================
    // 8. INTEGRATE VELOCITY STATE (CRITICAL)
    // =========================================================================
    shaped_vel_xy_ += shaped_accel_xy_ * dt;

    // =========================================================================
    // 9. VERTICAL CONTROL (AP_Follow style)
    // =========================================================================
    double z_error = param_follow_height_ - drone.alt_rel;
    
    // Use scalar sqrt_controller with VELOCITY limit (not accel)
    double vel_target_z = ap_control::sqrt_controller(
        std::abs(z_error),       // Use magnitude
        param_kp_pos_,           // Position P gain
        CMD_VEL_VERTICAL_MAX,    // VELOCITY limit (not accel!)
        dt);
    
    // Restore sign
    if (z_error < 0.0) {
        vel_target_z = -vel_target_z;
    }
    
    // Jerk limiting - shape velocity to acceleration
    ap_control::shape_vel_accel(
        vel_target_z,
        0.0,                     // No accel feedforward
        shaped_vel_z_,
        shaped_accel_z_,
        -param_accel_max_d_,
        param_accel_max_d_,
        param_jerk_max_d_,
        dt,
        true
    );
    
    // Integrate acceleration to get velocity
    shaped_vel_z_ += shaped_accel_z_ * dt;

    // =========================================================================
    // 10. TRANSFORM TO BODY FRAME
    // =========================================================================
    double cp = std::cos(drone.yaw);
    double sp = std::sin(drone.yaw);
    
    cmd.vel_forward =  shaped_vel_xy_.x() * cp + shaped_vel_xy_.y() * sp;
    cmd.vel_left    = -shaped_vel_xy_.x() * sp + shaped_vel_xy_.y() * cp;
    cmd.vel_up      =  shaped_vel_z_;

    // =========================================================================
    // 11. YAW CONTROL (will be set by calculate_yaw_rate)
    // =========================================================================
    cmd.yaw_rate = 0.0;  // Will be set by calculate_yaw_rate()

    return cmd;
}



/**
 * @brief Calculate yaw rate command with dual-mode control
 * 
 * Two control modes blended together:
 * 1. Gimbal Tracking (Reactive): Keep target centered in camera view
 *    - Uses gimbal pan error feedback
 *    - High precision for manual control
 * 
 * 2. Heading Follow (Predictive): Smoothly track target's heading
 *    - Follows target movement direction
 *    - Provides cinematic motion
 * 
 * Blend weight controlled by param_heading_blend_weight_
 * 
 * @param drone Current drone state
 * @param target Current target state
 * @param dt Time delta
 * @return Yaw rate command (rad/s) with kinematic shaping applied
 */
double SmartFollowNode::calculate_yaw_rate(const DroneState& drone,
                                          const TargetState& target,
                                          double dt) {
    double yaw_rate_gimbal = 0.0;
    double yaw_rate_heading = 0.0;

    double pan_error;
    if (gimbal_tracker_.get_error(pan_error, this->now())) {
        if (std::abs(pan_error) > param_deadzone_) {
            yaw_rate_gimbal = -param_kp_yaw_ * pan_error;
        }
    }
    
    if (param_heading_blend_enable_ && target.valid) {
        double target_speed = target.vel_enu.head<2>().norm();
        if (target_speed > SPEED_THRESHOLD_OFFSET) {
            double heading_error = target.heading_rad - drone.yaw;
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
            yaw_rate_heading = param_heading_follow_kp_ * heading_error;
        }
    }
    
    double blend_weight = param_heading_blend_enable_ ? param_heading_blend_weight_ : 0.0;
    double yaw_rate_target = (1.0 - blend_weight) * yaw_rate_gimbal + 
                            blend_weight * yaw_rate_heading;

    if (dt > CONTROL_LOOP_DT_MIN && dt < CONTROL_LOOP_DT_MAX) {
        ap_control::shape_vel_accel(
            yaw_rate_target,
            0.0,
            shaped_yaw_rate_,
            shaped_yaw_accel_,
            -param_accel_max_h_,
            param_accel_max_h_,
            param_jerk_max_h_,
            dt,
            true
        );
        // Remove acceleration integration to get pure velocity shaping
        // shaped_yaw_rate_ += shaped_yaw_accel_ * dt;
    }

    return shaped_yaw_rate_;
}

// --- OFFSET CALCULATION ---

Eigen::Vector2d SmartFollowNode::calculate_offset_enu(const TargetState& target, double desired_dist) {
    Eigen::Vector2d offset = Eigen::Vector2d::Zero();
    double target_speed = target.vel_enu.head<2>().norm();
    
    switch (param_offset_type_) {
        case OffsetType::NED:
            offset.x() = param_offset_ned_.y();
            offset.y() = param_offset_ned_.x();
            break;
            
        case OffsetType::RELATIVE:
            if (target_speed > SPEED_THRESHOLD_OFFSET) {
                Eigen::Vector2d offset_body;
                offset_body.x() = param_offset_ned_.y();
                offset_body.y() = param_offset_ned_.x();
                offset = rotate_vector_2d(offset_body, target.heading_rad);
            } else {
                offset.x() = param_offset_ned_.y();
                offset.y() = param_offset_ned_.x();
            }
            break;
            
        case OffsetType::VELOCITY:
        default:
            if (target_speed > SPEED_THRESHOLD_OFFSET) {
                offset = -target.vel_enu.head<2>() / target_speed * desired_dist;
            }
            break;
    }
    
    return offset;
}

Eigen::Vector2d SmartFollowNode::rotate_vector_2d(const Eigen::Vector2d& vec, double angle_rad) const {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    Eigen::Vector2d rotated;
    rotated.x() = vec.x() * c - vec.y() * s;
    rotated.y() = vec.x() * s + vec.y() * c;
    return rotated;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Publish velocity command to ArduPilot via DDS
 * @param cmd Navigation command in body frame
 */
void SmartFollowNode::publish_cmd(const NavCommand& cmd) {
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
    NavCommand stop = {0, 0, 0, 0};
    publish_cmd(stop);
}

/**
 * @brief Check if prediction error exceeds safe thresholds (AP_Follow algorithm)
 * 
 * Compares current measurement with last prediction to detect:
 * - GPS glitches (large position jumps)
 * - Unreasonable velocities/accelerations
 * - Filter divergence
 * 
 * Thresholds calculated from kinematic limits and timeout period.
 * 
 * @param target Current target state
 * @return true if error is too large (should reset filters)
 */
bool SmartFollowNode::estimate_error_too_large(const TargetState& target) const {
    if (!target.valid) {
        return false;
    }
    
    // Skip check if we don't have a previous estimate yet (first few updates)
    if (last_estimate_pos_enu_.norm() < 0.1) {
        return false;  // No previous estimate to compare against
    }
    
    const double timeout_sec = param_timeout_;
    const double pos_thresh_horiz_m = param_accel_max_ne_ * 
                                     std::pow(timeout_sec * 0.5, 2);
    const double pos_thresh_vert_m = param_accel_max_d_ * 
                                    std::pow(timeout_sec * 0.5, 2);
    
    const double vel_thresh_horiz_ms = calc_max_velocity_change(
        param_accel_max_ne_, param_jerk_max_ne_, timeout_sec);
    const double vel_thresh_vert_ms = calc_max_velocity_change(
        param_accel_max_d_, param_jerk_max_d_, timeout_sec);
    
    Eigen::Vector3d pos_error = last_estimate_pos_enu_ - target.pos_enu;
    Eigen::Vector3d vel_error = last_estimate_vel_enu_ - target.vel_enu;
    
    double pos_error_horiz = pos_error.head<2>().norm();
    double vel_error_horiz = vel_error.head<2>().norm();
    double pos_error_vert = std::abs(pos_error.z());
    double vel_error_vert = std::abs(vel_error.z());
    
    bool pos_horiz_bad = pos_error_horiz > pos_thresh_horiz_m;
    bool vel_horiz_bad = vel_error_horiz > vel_thresh_horiz_ms;
    bool pos_vert_bad = pos_error_vert > pos_thresh_vert_m;
    bool vel_vert_bad = vel_error_vert > vel_thresh_vert_ms;
    
    double target_vel_horiz = target.vel_enu.head<2>().norm();
    double target_accel_horiz = target.accel_enu.head<2>().norm();
    double target_accel_vert = std::abs(target.accel_enu.z());
    
    bool velocity_unreasonable = target_vel_horiz > MAX_TARGET_VELOCITY;
    bool accel_unreasonable = (target_accel_horiz > MAX_TARGET_ACCELERATION) ||
                             (target_accel_vert > MAX_TARGET_ACCELERATION);
    
    bool error_detected = pos_horiz_bad || vel_horiz_bad || pos_vert_bad || vel_vert_bad ||
                          velocity_unreasonable || accel_unreasonable;
    
    // Debug logging when error detected
    if (error_detected) {
        RCLCPP_WARN(get_logger(),
                   "Estimate error: pos_h=%.2f/%.2f vel_h=%.2f/%.2f pos_v=%.2f/%.2f vel_v=%.2f/%.2f",
                   pos_error_horiz, pos_thresh_horiz_m,
                   vel_error_horiz, vel_thresh_horiz_ms,
                   pos_error_vert, pos_thresh_vert_m,
                   vel_error_vert, vel_thresh_vert_ms);
    }
    
    return error_detected;
}

double SmartFollowNode::calc_max_velocity_change(double accel_max, double jerk_max, double timeout_sec) const {
    const double t_jerk = accel_max / jerk_max;
    const double t_total_jerk = 2.0 * t_jerk;
    
    if (timeout_sec >= t_total_jerk) {
        const double t_const = timeout_sec - t_total_jerk;
        const double delta_v_jerk = 0.5 * accel_max * t_jerk;
        const double delta_v_const = accel_max * t_const;
        return 2.0 * delta_v_jerk + delta_v_const;
    } else {
        const double t_half = timeout_sec * 0.5;
        return 0.5 * jerk_max * std::pow(t_half, 2);
    }
}

void SmartFollowNode::reset_shaping_state() {
    shaped_vel_xy_.setZero();
    shaped_accel_xy_.setZero();
    shaped_vel_z_ = 0.0;
    shaped_accel_z_ = 0.0;
    shaped_yaw_rate_ = 0.0;
    shaped_yaw_accel_ = 0.0;
    follow_estimator_.reset();
}

// ============================================================================
// GPS HEALTH MONITORING
// ============================================================================

/**
 * @brief Update GPS health metrics from NavSatFix message
 */
void SmartFollowNode::update_gps_health(const sensor_msgs::msg::NavSatFix::SharedPtr msg, 
                                        GPSHealth& health) {
    // Extract position covariance (horizontal accuracy)
    // NavSatFix covariance matrix: [x, y, z] positions
    // We use horizontal (x,y) components
    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        double cov_x = msg->position_covariance[0];
        double cov_y = msg->position_covariance[4];
        health.covariance = std::max(cov_x, cov_y);
        health.horizontal_accuracy = std::sqrt(health.covariance);
        health.has_covariance = true;
    } else {
        // No covariance data available from sensor
        health.has_covariance = false;
    }
    
    health.last_update = this->now();
    health.frozen_detected = false;  // Reset, will be checked separately
}

/**
 * @brief Detect frozen GPS stream (no updates)
 */
void SmartFollowNode::check_gps_frozen(GPSHealth& health, const rclcpp::Time& now) {
    double time_since_update = (now - health.last_update).seconds();
    
    if (time_since_update > GPS_FROZEN_TIMEOUT) {
        health.frozen_detected = true;
    }
}

/**
 * @brief Check if drone altitude is within safe limits
 */
bool SmartFollowNode::check_altitude_safety(const DroneState& drone) const {
    // Check relative altitude (AGL - Above Ground Level)
    if (drone.alt_rel > MAX_ALTITUDE_AGL) {
        return false;
    }
    
    // Check absolute altitude (MSL - Mean Sea Level)
    if (drone.alt_msl > MAX_ALTITUDE_MSL) {
        return false;
    }
    
    return true;
}

/**
 * @brief Emergency descent when altitude limits exceeded
 */
void SmartFollowNode::emergency_descent() {
    if (emergency_descent_active_.load()) {
        return;  // Already descending
    }
    
    emergency_descent_active_.store(true);
    
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    
    // Controlled descent at maximum safe rate
    msg.twist.linear.x = 0.0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = -CMD_VEL_VERTICAL_MAX;  // Descend at max rate
    msg.twist.angular.z = 0.0;
    
    pub_cmd_vel_->publish(msg);
    
    RCLCPP_ERROR(get_logger(), 
                "EMERGENCY DESCENT ACTIVE - Descending at %.1f m/s",
                CMD_VEL_VERTICAL_MAX);
}

/**
 * @brief Publish diagnostic status (1Hz)
 */
void SmartFollowNode::publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "smart_follow_node";
    diag.hardware_id = "ardupilot_drone";
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Determine overall status
    bool system_healthy = true;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    
    // Check origin
    if (!origin_manager_.is_ready()) {
        system_healthy = false;
        errors.push_back("EKF origin not set");
    }
    
    // Check drone GPS
    if (!drone_state_.gps_valid) {
        system_healthy = false;
        errors.push_back("Drone GPS invalid");
    } else if (!drone_gps_health_.is_healthy()) {
        warnings.push_back("Drone GPS accuracy degraded");
    }
    
    // Check target GPS
    if (!target_state_.valid) {
        warnings.push_back("No target GPS data");
    } else if (!target_gps_health_.is_healthy()) {
        warnings.push_back("Target GPS accuracy degraded");
    }
    
    // Check altitude safety
    if (!check_altitude_safety(drone_state_)) {
        system_healthy = false;
        errors.push_back("Altitude limit exceeded");
    }
    
    // Check target timeout
    double time_since_target = (this->now() - target_state_.last_update).seconds();
    if (target_state_.valid && time_since_target > param_timeout_) {
        warnings.push_back("Target GPS timeout");
    }
    
    // Set diagnostic level
    if (!system_healthy) {
        diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag.message = "System errors detected";
    } else if (!warnings.empty()) {
        diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diag.message = "System warnings active";
    } else {
        diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diag.message = "System healthy";
    }
    
    // Add detailed key-value pairs
    auto add_kv = [&diag](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        diag.values.push_back(kv);
    };
    
    add_kv("armed", is_armed_.load() ? "true" : "false");
    add_kv("guided", is_guided_.load() ? "true" : "false");
    add_kv("origin_ready", origin_manager_.is_ready() ? "true" : "false");
    add_kv("drone_gps_valid", drone_state_.gps_valid ? "true" : "false");
    add_kv("drone_gps_accuracy", std::to_string(drone_gps_health_.horizontal_accuracy) + "m");
    add_kv("drone_altitude_rel", std::to_string(drone_state_.alt_rel) + "m");
    add_kv("drone_altitude_msl", std::to_string(drone_state_.alt_msl) + "m");
    add_kv("target_valid", target_state_.valid ? "true" : "false");
    add_kv("target_gps_accuracy", std::to_string(target_gps_health_.horizontal_accuracy) + "m");
    add_kv("target_timeout", std::to_string(time_since_target) + "s");
    add_kv("filter_enabled", param_filter_enable_ ? "true" : "false");
    add_kv("terrain_follow", param_terrain_follow_enable_ ? "true" : "false");
    add_kv("emergency_descent", emergency_descent_active_.load() ? "true" : "false");
    
    // Add errors and warnings
    for (size_t i = 0; i < errors.size(); ++i) {
        add_kv("error_" + std::to_string(i), errors[i]);
    }
    for (size_t i = 0; i < warnings.size(); ++i) {
        add_kv("warning_" + std::to_string(i), warnings[i]);
    }
    
    pub_diagnostics_->publish(diag);
}