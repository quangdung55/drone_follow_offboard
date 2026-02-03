#include "drone_offboard/smart_follow_node.hpp"

using namespace std::chrono_literals;

SmartFollowNode::SmartFollowNode() : Node("smart_follow_node") {
    setup_parameters();
    setup_qos();
    setup_pub_sub();

    // Initialize Alpha-Beta Filters
    filter_vel_n_.reset(0.0);
    filter_vel_e_.reset(0.0);
    filter_alt_.reset(0.0);

    // Initialize last control time
    last_control_time_ = this->now();
    last_gimbal_msg_ = this->now();     // Initialize with Node's clock
    target_.last_update = this->now();  // Initialize with Node's clock
    
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
    this->declare_parameter("follow_dist", 5.0);         // Giữ khoảng cách 5m
    this->declare_parameter("follow_height", 3.0);       // Bay cao hơn target 3m (Terrain Follow)
    this->declare_parameter("prediction_time", 0.5);     // Dự đoán trước 0.5s
    this->declare_parameter("timeout", 3.0);             // AP_FOLLOW style timeout (giây)

    // PID & FeedForward Gains
    this->declare_parameter("kp_pos", 0.1);              // AP_FOLLOW_POS_P_DEFAULT = 0.1
    this->declare_parameter("kp_vel_z", 1.5);            // Hệ số P vị trí Z (riêng biệt)
    this->declare_parameter("kp_yaw", 0.05);             // Hệ số P quay (Gimbal)
    this->declare_parameter("ff_lateral_gain", 0.85);    // Lateral FF (0.0 - 1.0)
    this->declare_parameter("ff_centripetal_gain", 2.0); // Centripetal compensation

    // Constraints - Tách riêng cho từng trục như AP_Follow
    // FOLL_ACCEL_NE: Horizontal acceleration limit
    this->declare_parameter("accel_max_ne", 2.5);        // m/s² (AP default: 2.5)
    // FOLL_JERK_NE: Horizontal jerk limit  
    this->declare_parameter("jerk_max_ne", 5.0);         // m/s³ (AP default: 5.0)
    // FOLL_ACCEL_D: Vertical acceleration limit
    this->declare_parameter("accel_max_d", 2.5);         // m/s² (AP default: 2.5)
    // FOLL_JERK_D: Vertical jerk limit
    this->declare_parameter("jerk_max_d", 5.0);          // m/s³ (AP default: 5.0)
    // FOLL_ACCEL_H: Heading angular acceleration limit
    this->declare_parameter("accel_max_h", 90.0);        // deg/s² (AP default: 90)
    // FOLL_JERK_H: Heading angular jerk limit
    this->declare_parameter("jerk_max_h", 360.0);        // deg/s³ (AP default: 360)
    
    this->declare_parameter("gimbal_deadzone", 5.0);     // Vùng chết Gimbal (độ)

    // Alpha-Beta Filter Params
    this->declare_parameter("filter_enable", true);         // Enable/disable filtering
    this->declare_parameter("filter_alpha", 0.7);        // Tin vào đo đạc 60%
    this->declare_parameter("filter_beta", 0.1);         // Cập nhật vận tốc chậm hơn

    // Advanced Features - Adaptive Distance
    this->declare_parameter("adaptive_distance_enable", false);
    this->declare_parameter("follow_dist_min", 3.0);
    this->declare_parameter("follow_dist_max", 15.0);
    this->declare_parameter("distance_speed_gain", 1.5);

    // Advanced Features - Heading Blending
    this->declare_parameter("heading_blend_enable", false);
    this->declare_parameter("heading_blend_weight", 0.3);
    this->declare_parameter("heading_follow_kp", 0.5);

    // Advanced Features - Terrain Following
    this->declare_parameter("terrain_follow_enable", true);
    
    // Advanced Features - Offset Mode (AP_Follow style)
    this->declare_parameter("offset_type", 2);           // 0=NED, 1=Relative, 2=Velocity (default)
    this->declare_parameter("offset_north", 0.0);        // Static offset North (m)
    this->declare_parameter("offset_east", 0.0);         // Static offset East (m)  
    this->declare_parameter("offset_down", 0.0);         // Static offset Down (m)
    
    // Advanced Features - Jitter Correction (AP_Follow style)
    this->declare_parameter("jitter_correction_enable", true);  // Enable timestamp jitter correction
    this->declare_parameter("jitter_max_lag_ms", 500);         // Max transport lag (ms)
    this->declare_parameter("jitter_convergence_loops", 100);   // Samples for convergence
    
    // Advanced Features - Adaptive Beta Filter (Fast response on large errors)
    this->declare_parameter("filter_beta_fast", 0.1);           // Beta when residual > threshold
    this->declare_parameter("filter_beta_slow", 0.05);           // Beta when residual <= threshold
    this->declare_parameter("filter_residual_threshold", 2.0);  // Threshold (m) to switch to fast beta

    // Cache params để truy xuất nhanh
    param_follow_dist_ = this->get_parameter("follow_dist").as_double();
    param_follow_height_ = this->get_parameter("follow_height").as_double();
    param_pred_time_ = this->get_parameter("prediction_time").as_double();
    param_timeout_ = this->get_parameter("timeout").as_double();
    param_kp_pos_ = this->get_parameter("kp_pos").as_double();
    param_kp_vel_z_ = this->get_parameter("kp_vel_z").as_double();
    param_kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    param_ff_lateral_gain_ = this->get_parameter("ff_lateral_gain").as_double();
    param_ff_centripetal_gain_ = this->get_parameter("ff_centripetal_gain").as_double();
    
    // Kinematic constraints - tách riêng cho từng trục (AP_Follow style)
    param_accel_max_ne_ = this->get_parameter("accel_max_ne").as_double();
    param_jerk_max_ne_ = this->get_parameter("jerk_max_ne").as_double();
    param_accel_max_d_ = this->get_parameter("accel_max_d").as_double();
    param_jerk_max_d_ = this->get_parameter("jerk_max_d").as_double();
    param_accel_max_h_ = this->get_parameter("accel_max_h").as_double() * DEG_TO_RAD; // Convert to rad/s²
    param_jerk_max_h_ = this->get_parameter("jerk_max_h").as_double() * DEG_TO_RAD;  // Convert to rad/s³
    
    param_deadzone_ = this->get_parameter("gimbal_deadzone").as_double();
    param_filter_enable_ = this->get_parameter("filter_enable").as_bool();
    param_filter_alpha_ = this->get_parameter("filter_alpha").as_double();
    param_filter_beta_ = this->get_parameter("filter_beta").as_double();
    param_filter_beta_fast_ = this->get_parameter("filter_beta_fast").as_double();
    param_filter_beta_slow_ = this->get_parameter("filter_beta_slow").as_double();
    param_filter_residual_threshold_ = this->get_parameter("filter_residual_threshold").as_double();
    
    // Cache advanced features
    param_adaptive_dist_enable_ = this->get_parameter("adaptive_distance_enable").as_bool();
    param_follow_dist_min_ = this->get_parameter("follow_dist_min").as_double();
    param_follow_dist_max_ = this->get_parameter("follow_dist_max").as_double();
    param_dist_speed_gain_ = this->get_parameter("distance_speed_gain").as_double();
    
    param_heading_blend_enable_ = this->get_parameter("heading_blend_enable").as_bool();
    param_heading_blend_weight_ = this->get_parameter("heading_blend_weight").as_double();
    param_heading_follow_kp_ = this->get_parameter("heading_follow_kp").as_double();
    
    param_terrain_follow_enable_ = this->get_parameter("terrain_follow_enable").as_bool();
    
    // Cache offset mode parameters
    int offset_type_int = this->get_parameter("offset_type").as_int();
    param_offset_type_ = static_cast<OffsetType>(offset_type_int);
    param_offset_ned_.x() = this->get_parameter("offset_north").as_double();
    param_offset_ned_.y() = this->get_parameter("offset_east").as_double();
    param_offset_ned_.z() = this->get_parameter("offset_down").as_double();
    
    // Cache jitter correction parameters
    param_jitter_correction_enable_ = this->get_parameter("jitter_correction_enable").as_bool();
    param_jitter_max_lag_ms_ = this->get_parameter("jitter_max_lag_ms").as_int();
    param_jitter_convergence_loops_ = this->get_parameter("jitter_convergence_loops").as_int();
    
    // Initialize jitter correction with parameters
    jitter_correction_.emplace(param_jitter_max_lag_ms_, param_jitter_convergence_loops_);
}

void SmartFollowNode::setup_qos() {
    // QoS Settings: Best Effort là BẮT BUỘC cho ArduPilot DDS (UDP)
    qos_best_effort_ = rclcpp::SensorDataQoS();
    qos_best_effort_.keep_last(1);
    qos_best_effort_.best_effort();
    qos_best_effort_.durability_volatile();
}

void SmartFollowNode::setup_pub_sub() {
    // === SUBSCRIBERS ===
    
    // Topic EKF Global Origin - GỐC TỌA ĐỘ CỐ ĐỊNG (QUAN TRỌNG NHẤT!)
    // Đây là origin WGS84 mà EKF của ArduPilot chọn
    // Tất cả local pose đều quy chiếu về điểm này
    sub_global_origin_ = create_subscription<geographic_msgs::msg::GeoPointStamped>(
        "/ap/gps_global_origin/filtered", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_global_origin, this, std::placeholders::_1));

    // Topic GPS native của ArduPilot (dùng để convert target GPS về ENU)
    sub_drone_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ap/navsat", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_drone_gps, this, std::placeholders::_1));

    // Topic Pose (Local Position & Orientation) của ArduPilot
    sub_drone_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ap/pose/filtered", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_drone_pose, this, std::placeholders::_1));

    // Topic GPS của mục tiêu (Từ App điện thoại hoặc Module GPS rời)
    sub_target_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target/gps", qos_best_effort_,
        std::bind(&SmartFollowNode::cb_target_gps, this, std::placeholders::_1));

    // Topic lỗi góc Gimbal: x = pan error (deg), y = tilt error (deg)
    sub_gimbal_ = create_subscription<geometry_msgs::msg::Point>(
        "/gimbal/angle_error", 10,
        std::bind(&SmartFollowNode::cb_gimbal_angle, this, std::placeholders::_1));

    // ArduPilot DDS Native: Flight Mode status
    sub_status_ = create_subscription<ardupilot_msgs::msg::Status>(
        "/ap/status", 10,
        std::bind(&SmartFollowNode::cb_status, this, std::placeholders::_1));

    // === PUBLISHER ===
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ap/cmd_vel", 10);

    // Timer Loop (50Hz - 20ms) - Tăng từ 20Hz lên 50Hz cho mượt hơn
    timer_ = create_wall_timer(20ms, std::bind(&SmartFollowNode::control_loop, this));
}

// --- CALLBACK IMPLEMENTATION ---

void SmartFollowNode::cb_global_origin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {
    // Receive fixed ENU origin from ArduPilot EKF
    // Only update once (origin doesn't change during flight)
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!origin_ready_) {
        origin_lat_ = msg->position.latitude;
        origin_lon_ = msg->position.longitude;
        origin_alt_ = msg->position.altitude;
        origin_ready_ = true;
        RCLCPP_INFO(get_logger(), "EKF Origin Received: lat=%.7f, lon=%.7f, alt=%.2f",
                    origin_lat_, origin_lon_, origin_alt_);
    }
}

void SmartFollowNode::cb_drone_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    drone_lat_ = msg->latitude;
    drone_lon_ = msg->longitude;
    drone_alt_ = msg->altitude;  // Absolute altitude (MSL)
    
    // Must have origin before conversion!
    if (!origin_ready_) return;
    
    // Convert GPS -> Map-ENU frame
    double e, n;
    gps_to_map_enu_internal(msg->latitude, msg->longitude, e, n);
    
    drone_map_.pos_enu.x() = e;  // East
    drone_map_.pos_enu.y() = n;  // North
    drone_map_.pos_enu.z() = msg->altitude - origin_alt_;  // Up (relative to origin)
    
    drone_ready_ = true;
}

void SmartFollowNode::cb_drone_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Get Relative Altitude (from ArduPilot EKF - relative to Home)
    drone_relative_alt_ = msg->pose.position.z;

    // Extract Yaw from Quaternion using Eigen (more accurate than tf2)
    Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    
    // Extract yaw from quaternion (avoids gimbal lock)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    drone_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    
    pose_ready_ = true;
}

void SmartFollowNode::cb_target_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Must have origin before conversion!
    if (!origin_ready_) return;
    
    // Extract timestamps for jitter correction
    // Offboard timestamp: from GPS message header (sensor time)
    uint64_t offboard_usec = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000ULL;
    
    // Local timestamp: current system time (arrival time)
    uint64_t local_usec = this->now().nanoseconds() / 1000ULL;
    
    // Apply jitter correction if enabled
    uint64_t corrected_usec;
    if (param_jitter_correction_enable_ && offboard_usec > 0 && jitter_correction_.has_value()) {
        corrected_usec = jitter_correction_->correct_offboard_timestamp_usec(offboard_usec, local_usec);
    } else {
        // Fallback: use local timestamp if jitter correction disabled or no valid offboard timestamp
        corrected_usec = local_usec;
    }
    
    // Calculate dt using corrected timestamp
    double now_sec = corrected_usec / 1000000.0;  // Convert µs to seconds
    double dt = target_.valid ? (now_sec - target_.last_update.seconds()) : 0.0;
    
    // Convert GPS -> Map-ENU frame
    double e, n;
    gps_to_map_enu_internal(msg->latitude, msg->longitude, e, n);
    Eigen::Vector2d new_pos(e, n);

    if (target_.valid && dt > GPS_DT_MIN_VALID && dt < GPS_DT_MAX_VALID) {
        // 1. Calculate raw velocity from position delta in Map frame
        Eigen::Vector2d delta = new_pos - target_map_.pos_enu.head<2>();
        Eigen::Vector2d raw_vel = delta / dt;

        // 2. Apply filter if enabled, otherwise use raw velocity
        double old_vel_e, old_vel_n;
        if (param_filter_enable_) {
            // Alpha-Beta Filter for velocity (with adaptive beta for fast response)
            filter_vel_e_.update_adaptive(raw_vel.x(), dt, param_filter_alpha_, 
                                          param_filter_beta_fast_, param_filter_beta_slow_, 
                                          param_filter_residual_threshold_);
            filter_vel_n_.update_adaptive(raw_vel.y(), dt, param_filter_alpha_, 
                                          param_filter_beta_fast_, param_filter_beta_slow_, 
                                          param_filter_residual_threshold_);
            
            // Get filtered velocity
            old_vel_e = target_map_.vel_enu.x();
            old_vel_n = target_map_.vel_enu.y();
            target_map_.vel_enu.x() = filter_vel_e_.x;  // East
            target_map_.vel_enu.y() = filter_vel_n_.x;  // North
        } else {
            // Use raw velocity directly (no filtering)
            old_vel_e = target_map_.vel_enu.x();
            old_vel_n = target_map_.vel_enu.y();
            target_map_.vel_enu.x() = raw_vel.x();
            target_map_.vel_enu.y() = raw_vel.y();
        }

        // 3. Calculate acceleration from velocity change
        double instant_accel_e = (target_map_.vel_enu.x() - old_vel_e) / dt;
        double instant_accel_n = (target_map_.vel_enu.y() - old_vel_n) / dt;
        target_map_.accel_enu.x() = ACCEL_FILTER_ALPHA * instant_accel_e + (1.0 - ACCEL_FILTER_ALPHA) * target_map_.accel_enu.x();
        target_map_.accel_enu.y() = ACCEL_FILTER_ALPHA * instant_accel_n + (1.0 - ACCEL_FILTER_ALPHA) * target_map_.accel_enu.y();
        
        // Clamp acceleration magnitude (avoid spikes from GPS noise)
        double accel_mag = target_map_.accel_enu.head<2>().norm();
        if (accel_mag > ACCEL_MAGNITUDE_CLAMP) {
            target_map_.accel_enu.head<2>() *= (ACCEL_MAGNITUDE_CLAMP / accel_mag);
        }

        // 4. Filter Altitude for Terrain Following (if filter enabled)
        if (param_filter_enable_) {
            filter_alt_.update(msg->altitude, dt, param_filter_alpha_, param_filter_beta_);
            target_.alt = filter_alt_.x;
            target_map_.pos_enu.z() = filter_alt_.x - origin_alt_;
        } else {
            target_.alt = msg->altitude;
            target_map_.pos_enu.z() = msg->altitude - origin_alt_;
        }

        // 5. Calculate Heading and Yaw Rate from filtered velocity
        double speed = target_map_.vel_enu.head<2>().norm();
        if (speed > SPEED_THRESHOLD_HEADING) {
            double new_heading = std::atan2(target_map_.vel_enu.y(), target_map_.vel_enu.x());
            
            // Calculate Yaw Rate (wrap angle properly)
            double diff_heading = new_heading - target_.heading_rad;
            if (diff_heading > M_PI) diff_heading -= 2.0 * M_PI;
            if (diff_heading < -M_PI) diff_heading += 2.0 * M_PI;
            
            // Low-pass filter yaw rate to avoid noise
            target_.yaw_rate = YAW_RATE_FILTER_ALPHA * (diff_heading / dt) + (1.0 - YAW_RATE_FILTER_ALPHA) * target_.yaw_rate;
            target_.heading_rad = new_heading;
        } else {
            target_.yaw_rate = 0.0;
        }
    } else {
        // First run - initialize
        filter_vel_n_.reset(0.0);
        filter_vel_e_.reset(0.0);
        filter_alt_.reset(msg->altitude);
        target_map_.pos_enu.z() = msg->altitude - origin_alt_;
    }

    // Update position trong Map frame
    target_map_.pos_enu.x() = new_pos.x();
    target_map_.pos_enu.y() = new_pos.y();
    
    // Lưu GPS raw cho reference
    target_.lat = msg->latitude;
    target_.lon = msg->longitude;
    
    // Store corrected timestamp (already calculated above)
    // Use node's clock type to ensure compatibility with this->now()
    target_.last_update = rclcpp::Time(corrected_usec * 1000, this->get_clock()->get_clock_type());
    target_.valid = true;
}

void SmartFollowNode::cb_gimbal_angle(const geometry_msgs::msg::Point::SharedPtr msg) {
    gimbal_pan_error_ = msg->x;
    last_gimbal_msg_ = this->now();
}

void SmartFollowNode::cb_status(const ardupilot_msgs::msg::Status::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 1.Update state Arm
    is_armed_ = msg->armed;

    // 2. Update Flight Mode
    // In ArduCopter: 0=Stabilize, 4=Guided, 9=Land, ...
    current_mode_int_ = msg->mode;
    
    // Check if mode is GUIDED (Mode 4)
    if (current_mode_int_ == 4) {
        is_guided_ = true;
    } else {
        is_guided_ = false;
    }

    // Log ra màn hình khi trạng thái thay đổi để dễ debug
    static bool last_armed = false;
    static int last_mode = -1;

    if (is_armed_ != last_armed || current_mode_int_ != last_mode) {
        RCLCPP_INFO(get_logger(), " Status Update -> Armed: %s | Mode: %d (Guided: %s)", 
            is_armed_ ? "YES" : "NO", 
            current_mode_int_,
            is_guided_ ? "YES" : "NO");
        last_armed = is_armed_;
        last_mode = current_mode_int_;
    }
}

// --- CORE LOGIC ---

void SmartFollowNode::gps_to_enu_delta(double lat_ref, double lon_ref, double lat_target, double lon_target, double& d_north, double& d_east) {
    double d_lat = (lat_target - lat_ref) * DEG_TO_RAD;
    double d_lon = (lon_target - lon_ref) * DEG_TO_RAD;
    double lat0_rad = lat_ref * DEG_TO_RAD;

    d_north = d_lat * R_EARTH;
    d_east  = d_lon * R_EARTH * std::cos(lat0_rad);
}

// Helper: Convert GPS -> Map-ENU (public interface with origin check)
void SmartFollowNode::gps_to_map_enu(double lat, double lon, double& e, double& n) const {
    if (!origin_ready_) {
        throw std::runtime_error("gps_to_map_enu called before origin is ready");
    }
    gps_to_map_enu_internal(lat, lon, e, n);
}

// Internal helper: Convert GPS -> Map-ENU (assumes origin is ready, no lock needed)
void SmartFollowNode::gps_to_map_enu_internal(double lat, double lon, double& e, double& n) const {
    double d_lat = (lat - origin_lat_) * DEG_TO_RAD;
    double d_lon = (lon - origin_lon_) * DEG_TO_RAD;
    double lat0_rad = origin_lat_ * DEG_TO_RAD;

    n = d_lat * R_EARTH;                      // North
    e = d_lon * R_EARTH * std::cos(lat0_rad); // East
}

NavCommand SmartFollowNode::calculate_kinematics() {
    NavCommand cmd = {0, 0, 0, 0};

    // Get current time for dt calculation
    double now_sec = this->now().seconds();
    double dt = now_sec - last_control_time_.seconds();
    if (dt <= CONTROL_LOOP_DT_MIN || dt > CONTROL_LOOP_DT_MAX) {
        dt = CONTROL_LOOP_DT_DEFAULT;  // Default to 50Hz if invalid
    }

    // --- 1. KINEMATIC PREDICTION WITH ACCELERATION ---
    // Dự đoán vị trí mục tiêu trong tương lai: P = P0 + V*t + 0.5*A*t²
    Eigen::Vector2d pred_offset;
    pred_offset.x() = target_map_.vel_enu.x() * param_pred_time_ 
                    + 0.5 * target_map_.accel_enu.x() * param_pred_time_ * param_pred_time_;
    pred_offset.y() = target_map_.vel_enu.y() * param_pred_time_ 
                    + 0.5 * target_map_.accel_enu.y() * param_pred_time_ * param_pred_time_;

    // --- 2. ADAPTIVE FOLLOW DISTANCE ---
    double desired_dist = param_follow_dist_;
    double target_speed = target_map_.vel_enu.head<2>().norm();
    
    if (param_adaptive_dist_enable_) {
        double raw_dist = param_follow_dist_min_ + param_dist_speed_gain_ * target_speed;
        raw_dist = std::clamp(raw_dist, param_follow_dist_min_, param_follow_dist_max_);
        
        // Low-pass filter to avoid jerky motion
        constexpr double ADAPTIVE_DIST_ALPHA = 0.1;
        filtered_desired_dist_ = ADAPTIVE_DIST_ALPHA * raw_dist + (1.0 - ADAPTIVE_DIST_ALPHA) * filtered_desired_dist_;
        desired_dist = filtered_desired_dist_;
    }

    // --- 3. CALCULATE FOLLOW OFFSET (AP_Follow style with multiple modes) ---
    Eigen::Vector2d target_offset = calculate_offset_enu(desired_dist);

    // --- 4. ERROR VECTOR CALCULATION (MAP-FRAME PURE) ---
    // No GPS math in control loop - both drone and target converted to Map-ENU in callbacks
    
    Eigen::Vector3d vec_error_enu = target_map_.pos_enu - drone_map_.pos_enu;
    
    // Add prediction and offset
    vec_error_enu.x() += pred_offset.x() + target_offset.x();  // East
    vec_error_enu.y() += pred_offset.y() + target_offset.y();  // North
    
    // --- 5. TERRAIN FOLLOWING (Altitude relative to target) ---
    if (param_terrain_follow_enable_) {
        // Fly above target by follow_height (in Map frame)
        vec_error_enu.z() = (target_map_.pos_enu.z() + param_follow_height_) - drone_map_.pos_enu.z();
    } else {
        // Fly at fixed altitude relative to home
        vec_error_enu.z() = param_follow_height_ - drone_relative_alt_;
    }

    // =========================================================================
    // ArduPilot Style: Sqrt Controller + Kinematic Shaping (NEW!)
    // Thay thế P controller đơn giản bằng sqrt controller để tránh overshoot
    // =========================================================================
    
    // --- 6. SQRT CONTROLLER FOR HORIZONTAL POSITION (XY) ---
    // k_v = jerk / accel (inner velocity loop gain)
    const double k_v_xy = param_jerk_max_ne_ / param_accel_max_ne_;
    
    // Position error in ENU frame
    Eigen::Vector2d pos_error_xy(vec_error_enu.x(), vec_error_enu.y());
    
    // Desired velocity = sqrt_controller(position_error) + feedforward_velocity
    Eigen::Vector2d vel_target_xy = ap_control::sqrt_controller(pos_error_xy, k_v_xy, param_accel_max_ne_, dt);
    
    // Add feedforward velocity from target
    double err_mag = pos_error_xy.norm();
    double ff_scale = std::clamp(err_mag / desired_dist, 0.0, 1.0);
    vel_target_xy += ff_scale * target_map_.vel_enu.head<2>();

    
    // Apply kinematic shaping (position → velocity → acceleration)
    // This uses shape_vel_accel_xy for smooth velocity tracking
    ap_control::shape_vel_accel_xy(
        vel_target_xy,                      // desired velocity
        target_map_.accel_enu.head<2>(),    // feedforward acceleration
        shaped_vel_xy_,                     // current shaped velocity (state)
        shaped_accel_xy_,                   // current shaped acceleration (modified)
        param_accel_max_ne_,                // accel_max
        param_jerk_max_ne_,                 // jerk_max
        dt,                                 // time step
        true                                // limit_total_accel
    );
    
    // Update shaped velocity from acceleration
    shaped_vel_xy_ += shaped_accel_xy_ * dt;
    
    // Save estimate for error checking (AP_Follow style)
    last_estimate_pos_enu_ = target_map_.pos_enu + Eigen::Vector3d(pred_offset.x(), pred_offset.y(), 0.0);
    last_estimate_vel_enu_ = target_map_.vel_enu;
    
    // --- 7. SQRT CONTROLLER FOR VERTICAL POSITION (Z) ---
    const double k_v_z = param_jerk_max_d_ / param_accel_max_d_;
    
    // Vertical velocity from sqrt controller
    double vel_target_z = ap_control::sqrt_controller(vec_error_enu.z(), k_v_z, param_accel_max_d_, dt);
    
    // Apply vertical shaping
    ap_control::shape_vel_accel(
        vel_target_z,                       // desired velocity
        0.0,                                // feedforward acceleration
        shaped_vel_z_,                      // current velocity (state)
        shaped_accel_z_,                    // current acceleration (modified)
        -param_accel_max_d_,                // accel_min
        param_accel_max_d_,                 // accel_max
        param_jerk_max_d_,                  // jerk_max
        dt,                                 // time step
        true                                // limit_total_accel
    );
    
    // Update shaped velocity
    shaped_vel_z_ += shaped_accel_z_ * dt;

    // --- 8. TRANSFORM TO BODY FRAME ---
    double cp = std::cos(drone_yaw_);
    double sp = std::sin(drone_yaw_);
    
    // ENU -> Body (Forward-Left-Up) using SHAPED velocities
    cmd.vel_forward =  shaped_vel_xy_.x() * cp + shaped_vel_xy_.y() * sp;
    cmd.vel_left    = -shaped_vel_xy_.x() * sp + shaped_vel_xy_.y() * cp;
    cmd.vel_up      =  shaped_vel_z_;

    return cmd;
}

// --- OFFSET CALCULATION (AP_Follow style) ---
Eigen::Vector2d SmartFollowNode::calculate_offset_enu(double desired_dist) {
    Eigen::Vector2d offset = Eigen::Vector2d::Zero();
    double target_speed = target_map_.vel_enu.head<2>().norm();
    
    switch (param_offset_type_) {
        case OffsetType::NED:
            // Static offset in NED frame (map-aligned)
            offset.x() = param_offset_ned_.y();  // East
            offset.y() = param_offset_ned_.x();  // North
            break;
            
        case OffsetType::RELATIVE:
            // Offset relative to target heading
            if (target_speed > SPEED_THRESHOLD_OFFSET) {
                // Rotate offset by target heading
                Eigen::Vector2d offset_body;
                offset_body.x() = param_offset_ned_.y();  // Right in body frame
                offset_body.y() = param_offset_ned_.x();  // Forward in body frame
                offset = rotate_vector_2d(offset_body, target_.heading_rad);
            } else {
                // No heading available, use NED offset
                offset.x() = param_offset_ned_.y();
                offset.y() = param_offset_ned_.x();
            }
            break;
            
        case OffsetType::VELOCITY:
        default:
            // Dynamic offset based on velocity direction (current behavior)
            if (target_speed > SPEED_THRESHOLD_OFFSET) {
                // Offset = -Velocity_Direction * Distance
                offset = -target_map_.vel_enu.head<2>() / target_speed * desired_dist;
            }
            break;
    }
    
    return offset;
}

// Helper: Rotate 2D vector by angle
Eigen::Vector2d SmartFollowNode::rotate_vector_2d(const Eigen::Vector2d& vec, double angle_rad) const {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    Eigen::Vector2d rotated;
    rotated.x() = vec.x() * c - vec.y() * s;
    rotated.y() = vec.x() * s + vec.y() * c;
    return rotated;
}

void SmartFollowNode::control_loop() {
    // Acquire lock for thread-safe access to shared state
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Safety Checks - Must have origin before controlling!
    if (!origin_ready_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
            "Waiting for EKF origin from /ap/gps_global_origin/filtered...");
        return;
    }
    
    if (!drone_ready_ || !pose_ready_ || !target_.valid) {
        stop_drone();
        return;
    }

    // Flight Mode Check (ArduPilot DDS Native)
    // If not GUIDED or not armed, reset state and don't send commands
    if (!is_guided_ || !is_armed_) {
        // Reset kinematic shaping state to avoid jerk when re-engaging
        reset_shaping_state();
        return;
    }

    // Target timeout (configurable, AP_Follow style)
    double time_since_update = (this->now() - target_.last_update).seconds();
    if (time_since_update > param_timeout_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
            "Lost Target Signal! (%.1fs > %.1fs timeout)", time_since_update, param_timeout_);
        stop_drone();
        return;
    }
    
    // AP_Follow style: Check if estimate error is too large
    if (estimate_error_too_large()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Estimate error too large, resetting...");
        // Reset filters and estimate
        filter_vel_n_.reset(0.0);
        filter_vel_e_.reset(0.0);
    }

    NavCommand nav = calculate_kinematics();

    // --- ADVANCED HYBRID YAW LOGIC WITH HEADING BLENDING ---
    double yaw_rate_gimbal = 0.0;
    double yaw_rate_heading = 0.0;

    // Component 1: Gimbal Tracking (Reactive - keeps target centered)
    bool gimbal_active = (this->now() - last_gimbal_msg_).seconds() < GIMBAL_TIMEOUT_SEC;
    if (gimbal_active && std::abs(gimbal_pan_error_) > param_deadzone_) {
        yaw_rate_gimbal = -param_kp_yaw_ * gimbal_pan_error_;
    }

    // Component 2: Heading Follow (Predictive - smooth cinematic motion)
    if (param_heading_blend_enable_ && target_.valid) {
        double target_speed = target_map_.vel_enu.head<2>().norm();
        
        if (target_speed > SPEED_THRESHOLD_OFFSET) {
            double heading_error = target_.heading_rad - drone_yaw_;
            // Wrap angle to [-PI, PI]
            heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
            yaw_rate_heading = param_heading_follow_kp_ * heading_error;
        }
    }

    // Blended Yaw Rate
    double blend_weight = param_heading_blend_enable_ ? param_heading_blend_weight_ : 0.0;
    double yaw_rate_target = (1.0 - blend_weight) * yaw_rate_gimbal + blend_weight * yaw_rate_heading;

    // --- ARDUPILOT STYLE YAW SHAPING ---
    // Apply angular velocity shaping with jerk limiting
    double now_sec = this->now().seconds();
    double dt = now_sec - last_control_time_.seconds();
    
    if (dt > CONTROL_LOOP_DT_MIN && dt < CONTROL_LOOP_DT_MAX) {
        ap_control::shape_vel_accel(
            yaw_rate_target,            // desired yaw rate
            0.0,                        // feedforward angular accel
            shaped_yaw_rate_,           // current yaw rate (state)
            shaped_yaw_accel_,          // current yaw accel (modified)
            -param_accel_max_h_,        // accel_min (rad/s²)
            param_accel_max_h_,         // accel_max (rad/s²)
            param_jerk_max_h_,          // jerk_max (rad/s³)
            dt,                         // time step
            true                        // limit_total_accel
        );
        
        // Update shaped yaw rate from acceleration
        shaped_yaw_rate_ += shaped_yaw_accel_ * dt;
    }
    
    nav.yaw_rate = shaped_yaw_rate_;
    
    last_control_time_ = this->now();

    // Safety Clamps (Velocity limits)
    nav.vel_forward = std::clamp(nav.vel_forward, -CMD_VEL_FORWARD_MAX, CMD_VEL_FORWARD_MAX);
    nav.vel_left    = std::clamp(nav.vel_left,    -CMD_VEL_LATERAL_MAX, CMD_VEL_LATERAL_MAX);
    nav.vel_up      = std::clamp(nav.vel_up,      -CMD_VEL_VERTICAL_MAX, CMD_VEL_VERTICAL_MAX);
    nav.yaw_rate    = std::clamp(nav.yaw_rate,    -CMD_YAW_RATE_MAX, CMD_YAW_RATE_MAX);

    publish_cmd(nav);
}

void SmartFollowNode::publish_cmd(const NavCommand& cmd) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    
    // IMPORTANT: "base_link" tells ArduPilot this is Body Frame velocity
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

// --- HELPER FUNCTIONS ---

double SmartFollowNode::shape_velocity(double desired_vel, double current_vel, double dt, double accel_max) {
    // Giới hạn tốc độ thay đổi vận tốc dựa trên gia tốc tối đa
    double delta_vel = desired_vel - current_vel;
    double max_delta = accel_max * dt;
    
    if (std::abs(delta_vel) > max_delta) {
        delta_vel = std::copysign(max_delta, delta_vel);
    }
    
    return current_vel + delta_vel;
}

void SmartFollowNode::apply_kinematic_limits(double& desired_vel, double& last_vel, double& last_acc, 
                                              double dt, double max_acc, double max_jerk) {
    // 2-Stage Kinematic Limiting: Jerk → Acceleration → Velocity
    // Ported từ ArduPilot AP_Follow
    
    // Stage 1: Calculate desired acceleration
    double desired_acc = (desired_vel - last_vel) / dt;
    
    // Stage 2: Limit jerk (rate of change of acceleration)
    double delta_acc = desired_acc - last_acc;
    double max_delta_acc = max_jerk * dt;
    
    if (std::abs(delta_acc) > max_delta_acc) {
        delta_acc = std::copysign(max_delta_acc, delta_acc);
    }
    double shaped_acc = last_acc + delta_acc;
    
    // Stage 3: Limit acceleration magnitude
    shaped_acc = std::clamp(shaped_acc, -max_acc, max_acc);
    
    // Stage 4: Apply shaped acceleration to velocity
    desired_vel = last_vel + shaped_acc * dt;
    
    // Save state for next iteration
    last_acc = shaped_acc;
    last_vel = desired_vel;
}

// AP_Follow::estimate_error_too_large() - Check if estimate is too far off
bool SmartFollowNode::estimate_error_too_large() const {
    // If no valid target, no need to check
    if (!target_.valid) {
        return false;
    }
    
    // AP_Follow style: Calculate position and velocity thresholds based on kinematic limits
    const double timeout_sec = param_timeout_;
    
    // Position threshold: max distance traveled under acceleration then deceleration
    // Distance = 0.5 * accel * (timeout/2)²  for each half
    const double pos_thresh_horiz_m = param_accel_max_ne_ * std::pow(timeout_sec * 0.5, 2);
    const double pos_thresh_vert_m = param_accel_max_d_ * std::pow(timeout_sec * 0.5, 2);
    
    // Velocity threshold: use trapezoidal profile calculation
    const double vel_thresh_horiz_ms = calc_max_velocity_change(
        param_accel_max_ne_, param_jerk_max_ne_, timeout_sec);
    const double vel_thresh_vert_ms = calc_max_velocity_change(
        param_accel_max_d_, param_jerk_max_d_, timeout_sec);
    
    // Calculate position error (difference between estimate and actual)
    Eigen::Vector3d pos_error = last_estimate_pos_enu_ - target_map_.pos_enu;
    Eigen::Vector3d vel_error = last_estimate_vel_enu_ - target_map_.vel_enu;
    
    // Check horizontal errors
    double pos_error_horiz = pos_error.head<2>().norm();
    double vel_error_horiz = vel_error.head<2>().norm();
    
    // Check vertical errors
    double pos_error_vert = std::abs(pos_error.z());
    double vel_error_vert = std::abs(vel_error.z());
    
    // Check against thresholds (AP_Follow logic)
    bool pos_horiz_bad = pos_error_horiz > pos_thresh_horiz_m;
    bool vel_horiz_bad = vel_error_horiz > vel_thresh_horiz_ms;
    bool pos_vert_bad = pos_error_vert > pos_thresh_vert_m;
    bool vel_vert_bad = vel_error_vert > vel_thresh_vert_ms;
    
    // Also check for unreasonable velocity/acceleration (original checks)
    double target_vel_horiz = target_map_.vel_enu.head<2>().norm();
    double target_accel_horiz = target_map_.accel_enu.head<2>().norm();
    double target_accel_vert = std::abs(target_map_.accel_enu.z());
    
    bool velocity_unreasonable = target_vel_horiz > MAX_TARGET_VELOCITY;
    bool accel_unreasonable = (target_accel_horiz > MAX_TARGET_ACCELERATION) || 
                              (target_accel_vert > MAX_TARGET_ACCELERATION);
    
    return pos_horiz_bad || vel_horiz_bad || pos_vert_bad || vel_vert_bad ||
           velocity_unreasonable || accel_unreasonable;
}

// AP_Follow::calc_max_velocity_change() - Calculate max velocity change under jerk-limited profile
double SmartFollowNode::calc_max_velocity_change(double accel_max, double jerk_max, double timeout_sec) const {
    // Time to ramp up acceleration from 0 to accel_max
    const double t_jerk = accel_max / jerk_max;
    const double t_total_jerk = 2.0 * t_jerk;
    
    if (timeout_sec >= t_total_jerk) {
        // Enough time for trapezoidal profile: ramp up, constant, ramp down
        const double t_const = timeout_sec - t_total_jerk;
        const double delta_v_jerk = 0.5 * accel_max * t_jerk;
        const double delta_v_const = accel_max * t_const;
        return 2.0 * delta_v_jerk + delta_v_const;
    } else {
        // Timeout too short: only triangular profile
        const double t_half = timeout_sec * 0.5;
        return 0.5 * jerk_max * std::pow(t_half, 2);
    }
}

// Reset all kinematic shaping state (called when disengaging)
void SmartFollowNode::reset_shaping_state() {
    // Reset ArduPilot shaping state
    shaped_vel_xy_.setZero();
    shaped_accel_xy_.setZero();
    shaped_vel_z_ = 0.0;
    shaped_accel_z_ = 0.0;
    shaped_yaw_rate_ = 0.0;
    shaped_yaw_accel_ = 0.0;
    
    // Reset follow estimator
    follow_estimator_.reset();
}