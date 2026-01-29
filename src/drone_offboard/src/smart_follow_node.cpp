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
    this->declare_parameter("filter_alpha", 0.6);        // Tin vào đo đạc 60%
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
    param_filter_alpha_ = this->get_parameter("filter_alpha").as_double();
    param_filter_beta_ = this->get_parameter("filter_beta").as_double();
    
    // Cache advanced features
    param_adaptive_dist_enable_ = this->get_parameter("adaptive_distance_enable").as_bool();
    param_follow_dist_min_ = this->get_parameter("follow_dist_min").as_double();
    param_follow_dist_max_ = this->get_parameter("follow_dist_max").as_double();
    param_dist_speed_gain_ = this->get_parameter("distance_speed_gain").as_double();
    
    param_heading_blend_enable_ = this->get_parameter("heading_blend_enable").as_bool();
    param_heading_blend_weight_ = this->get_parameter("heading_blend_weight").as_double();
    param_heading_follow_kp_ = this->get_parameter("heading_follow_kp").as_double();
    
    param_terrain_follow_enable_ = this->get_parameter("terrain_follow_enable").as_bool();
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
        "/ap/navsat/navsat_fix", qos_best_effort_,
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

    // ArduPilot DDS Native: Flight Mode (thay thế mavros)
    // Topic có thể là /ap/status/flight_mode hoặc tương tự
    sub_flight_mode_ = create_subscription<std_msgs::msg::String>(
        "/ap/status/flight_mode", 10,
        std::bind(&SmartFollowNode::cb_flight_mode, this, std::placeholders::_1));

    // ArduPilot DDS Native: Armed Status
    sub_armed_ = create_subscription<std_msgs::msg::Bool>(
        "/ap/status/armed", 10,
        std::bind(&SmartFollowNode::cb_armed_status, this, std::placeholders::_1));

    // === PUBLISHER ===
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ap/cmd_vel", 10);

    // Timer Loop (50Hz - 20ms) - Tăng từ 20Hz lên 50Hz cho mượt hơn
    timer_ = create_wall_timer(20ms, std::bind(&SmartFollowNode::control_loop, this));
}

// --- CALLBACK IMPLEMENTATION ---

void SmartFollowNode::cb_global_origin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {
    // Nhận gốc tọa độ ENU cố định từ EKF của ArduPilot
    // Chỉ cập nhật 1 lần (origin không đổi trong suốt flight)
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
    drone_lat_ = msg->latitude;
    drone_lon_ = msg->longitude;
    drone_alt_ = msg->altitude;  // Absolute altitude (MSL)
    
    // Phải có origin trước mới convert được!
    if (!origin_ready_) return;
    
    // Convert GPS → Map-ENU frame
    double e, n;
    gps_to_map_enu(msg->latitude, msg->longitude, e, n);
    
    drone_map_.pos_enu.x() = e;  // East
    drone_map_.pos_enu.y() = n;  // North
    drone_map_.pos_enu.z() = msg->altitude - origin_alt_;  // Up (relative to origin)
    
    drone_ready_ = true;
}

void SmartFollowNode::cb_drone_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Lấy Relative Altitude (từ EKF của ArduPilot - so với Home)
    drone_relative_alt_ = msg->pose.position.z;

    // Lấy Yaw từ Quaternion (sử dụng Eigen - chính xác hơn tf2)
    Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    
    // Extract yaw từ quaternion (tránh gimbal lock)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    drone_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    
    pose_ready_ = true;
}

void SmartFollowNode::cb_target_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // Phải có origin trước mới convert được!
    if (!origin_ready_) return;
    
    double now_sec = this->now().seconds();
    double dt = target_.valid ? (now_sec - target_.last_update.seconds()) : 0.0;
    
    // Convert GPS → Map-ENU frame
    double e, n;
    gps_to_map_enu(msg->latitude, msg->longitude, e, n);
    Eigen::Vector2d new_pos(e, n);

    if (target_.valid && dt > 0.01 && dt < 2.0) {
        // 1. Tính raw velocity từ position delta trong Map frame
        Eigen::Vector2d delta = new_pos - target_map_.pos_enu.head<2>();
        Eigen::Vector2d raw_vel = delta / dt;

        // 2. Alpha-Beta Filter cho velocity
        filter_vel_e_.update(raw_vel.x(), dt, param_filter_alpha_, param_filter_beta_);
        filter_vel_n_.update(raw_vel.y(), dt, param_filter_alpha_, param_filter_beta_);
        
        // Lấy velocity đã lọc
        double old_vel_e = target_map_.vel_enu.x();
        double old_vel_n = target_map_.vel_enu.y();
        target_map_.vel_enu.x() = filter_vel_e_.x;  // East
        target_map_.vel_enu.y() = filter_vel_n_.x;  // North

        // 3. Tính acceleration từ thay đổi velocity
        double alpha_accel = 0.15;
        double instant_accel_e = (target_map_.vel_enu.x() - old_vel_e) / dt;
        double instant_accel_n = (target_map_.vel_enu.y() - old_vel_n) / dt;
        target_map_.accel_enu.x() = alpha_accel * instant_accel_e + (1.0 - alpha_accel) * target_map_.accel_enu.x();
        target_map_.accel_enu.y() = alpha_accel * instant_accel_n + (1.0 - alpha_accel) * target_map_.accel_enu.y();
        
        // Giới hạn acceleration hợp lý (tránh spike từ GPS noise)
        double accel_mag = target_map_.accel_enu.head<2>().norm();
        if (accel_mag > 5.0) {
            target_map_.accel_enu.head<2>() *= (5.0 / accel_mag);
        }

        // 4. Filter Altitude cho Terrain Following
        filter_alt_.update(msg->altitude, dt, param_filter_alpha_, param_filter_beta_);
        target_.alt = filter_alt_.x;
        target_map_.pos_enu.z() = filter_alt_.x - origin_alt_;

        // 5. Tính Heading và Yaw Rate từ velocity đã lọc
        double speed = target_map_.vel_enu.head<2>().norm();
        if (speed > 0.5) {
            double new_heading = std::atan2(target_map_.vel_enu.y(), target_map_.vel_enu.x());
            
            // Tính Yaw Rate (wrap angle properly)
            double diff_heading = new_heading - target_.heading_rad;
            if (diff_heading > M_PI) diff_heading -= 2.0 * M_PI;
            if (diff_heading < -M_PI) diff_heading += 2.0 * M_PI;
            
            // Low-pass filter yaw rate để tránh noise
            double alpha_yaw = 0.3;
            target_.yaw_rate = alpha_yaw * (diff_heading / dt) + (1.0 - alpha_yaw) * target_.yaw_rate;
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
    target_.last_update = this->now();
    target_.valid = true;
}

void SmartFollowNode::cb_gimbal_angle(const geometry_msgs::msg::Point::SharedPtr msg) {
    gimbal_pan_error_ = msg->x;
    last_gimbal_msg_ = this->now();
}

void SmartFollowNode::cb_flight_mode(const std_msgs::msg::String::SharedPtr msg) {
    current_flight_mode_ = msg->data;
    // ArduPilot mode "GUIDED" cho phép velocity control
    is_guided_ = (msg->data == "GUIDED" || msg->data == "4");  // 4 = GUIDED mode number
}

void SmartFollowNode::cb_armed_status(const std_msgs::msg::Bool::SharedPtr msg) {
    is_armed_ = msg->data;
}

// --- CORE LOGIC ---

void SmartFollowNode::gps_to_enu_delta(double lat_ref, double lon_ref, double lat_target, double lon_target, double& d_north, double& d_east) {
    double d_lat = (lat_target - lat_ref) * DEG_TO_RAD;
    double d_lon = (lon_target - lon_ref) * DEG_TO_RAD;
    double lat0_rad = lat_ref * DEG_TO_RAD;

    d_north = d_lat * R_EARTH;
    d_east  = d_lon * R_EARTH * std::cos(lat0_rad);
}

// Helper: Convert GPS → Map-ENU (dùng origin cố định)
void SmartFollowNode::gps_to_map_enu(double lat, double lon, double& e, double& n) const {
    double d_lat = (lat - origin_lat_) * DEG_TO_RAD;
    double d_lon = (lon - origin_lon_) * DEG_TO_RAD;
    double lat0_rad = origin_lat_ * DEG_TO_RAD;

    n = d_lat * R_EARTH;                    // North
    e = d_lon * R_EARTH * std::cos(lat0_rad); // East
}

NavCommand SmartFollowNode::calculate_kinematics() {
    NavCommand cmd = {0, 0, 0, 0};

    // Get current time for dt calculation
    double now_sec = this->now().seconds();
    double dt = now_sec - last_control_time_.seconds();
    if (dt <= 0.001 || dt > 0.5) {
        dt = 0.02;  // Default to 50Hz if invalid
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
        
        // Low-pass filter để tránh giật
        double alpha_dist = 0.1;
        filtered_desired_dist_ = alpha_dist * raw_dist + (1.0 - alpha_dist) * filtered_desired_dist_;
        desired_dist = filtered_desired_dist_;
    }

    // --- 3. CALCULATE FOLLOW OFFSET (Bay sau lưng target) ---
    Eigen::Vector2d target_offset = Eigen::Vector2d::Zero();
    
    if (target_speed > 0.3) {
        // Offset = -Velocity_Direction * Distance
        target_offset = -target_map_.vel_enu.head<2>().normalized() * desired_dist;
    }
    // Khi target đứng yên: offset = 0 để hover tại chỗ

    // --- 4. ERROR VECTOR CALCULATION (MAP-FRAME PURE) ---
    // KHÔNG CÒN GPS MATH TRONG CONTROL LOOP
    // Cả drone và target đã được convert về Map-ENU trong callback
    
    Eigen::Vector3d vec_error_enu = target_map_.pos_enu - drone_map_.pos_enu;
    
    // Thêm prediction và offset
    vec_error_enu.x() += pred_offset.x() + target_offset.x();  // East
    vec_error_enu.y() += pred_offset.y() + target_offset.y();  // North
    
    // --- 5. TERRAIN FOLLOWING (Altitude relative to target) ---
    if (param_terrain_follow_enable_) {
        // Bay cao hơn target một khoảng follow_height (trong Map frame)
        vec_error_enu.z() = (target_map_.pos_enu.z() + param_follow_height_) - drone_map_.pos_enu.z();
    } else {
        // Bay theo độ cao cố định so với home
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

void SmartFollowNode::control_loop() {
    // Safety Checks - PHẢI có origin trước khi control!
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
    // Nếu không phải GUIDED hoặc chưa Arm, reset state và không gửi lệnh
    if (!is_guided_ || !is_armed_) {
        // Reset kinematic shaping state để tránh jerk khi engage lại
        reset_shaping_state();
        return;
    }

    // Timeout mục tiêu (configurable, AP_Follow style)
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
        // Reset filters và estimate
        filter_vel_n_.reset(0.0);
        filter_vel_e_.reset(0.0);
    }

    NavCommand nav = calculate_kinematics();

    // --- ADVANCED HYBRID YAW LOGIC WITH HEADING BLENDING ---
    double yaw_rate_gimbal = 0.0;
    double yaw_rate_heading = 0.0;

    // Component 1: Gimbal Tracking (Reactive - keeps target centered)
    bool gimbal_active = (this->now() - last_gimbal_msg_).seconds() < 0.5;
    if (gimbal_active && std::abs(gimbal_pan_error_) > param_deadzone_) {
        yaw_rate_gimbal = -param_kp_yaw_ * gimbal_pan_error_;
    }

    // Component 2: Heading Follow (Predictive - smooth cinematic motion)
    if (param_heading_blend_enable_ && target_.valid) {
        double target_speed = target_map_.vel_enu.head<2>().norm();
        
        if (target_speed > 0.3) {
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
    
    if (dt > 0.001 && dt < 0.5) {
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

    // Safety Clamps (Giới hạn vận tốc)
    nav.vel_forward = std::clamp(nav.vel_forward, -5.0, 5.0);
    nav.vel_left    = std::clamp(nav.vel_left,    -3.0, 3.0);
    nav.vel_up      = std::clamp(nav.vel_up,      -1.5, 1.5);
    nav.yaw_rate    = std::clamp(nav.yaw_rate,    -0.8, 0.8);

    publish_cmd(nav);
}

void SmartFollowNode::publish_cmd(const NavCommand& cmd) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    
    // QUAN TRỌNG: "base_link" báo cho ArduPilot đây là vận tốc Body Frame
    msg.header.frame_id = "base_link"; 

    msg.twist.linear.x = cmd.vel_forward;
    msg.twist.linear.y = cmd.vel_left;
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

// AP_Follow::estimate_error_too_large() - Kiểm tra xem estimate có quá sai lệch không
bool SmartFollowNode::estimate_error_too_large() const {
    // Nếu chưa có target valid, không cần check
    if (!target_.valid) {
        return false;
    }
    
    // Kiểm tra velocity của target có hợp lý không (dùng target_map_)
    double vel_horiz = target_map_.vel_enu.head<2>().norm();
    
    // Kiểm tra acceleration có vượt ngưỡng không (spike từ GPS noise)
    double accel_horiz = target_map_.accel_enu.head<2>().norm();
    double accel_vert = std::abs(target_map_.accel_enu.z());
    
    // Velocity quá lớn cho người/xe (> 50 m/s = 180 km/h)
    if (vel_horiz > 50.0) {
        return true;
    }
    
    // Acceleration quá lớn (> 10 m/s² - hơn 1G)
    if (accel_horiz > 10.0 || accel_vert > 10.0) {
        return true;
    }
    
    return false;
}

// AP_Follow::calc_max_velocity_change() - Tính max velocity change dưới jerk-limited profile
double SmartFollowNode::calc_max_velocity_change(double accel_max, double jerk_max, double timeout_sec) const {
    // Thời gian để ramp up acceleration từ 0 đến accel_max
    const double t_jerk = accel_max / jerk_max;
    const double t_total_jerk = 2.0 * t_jerk;
    
    if (timeout_sec >= t_total_jerk) {
        // Đủ thời gian cho trapezoidal profile: ramp up, constant, ramp down
        const double t_const = timeout_sec - t_total_jerk;
        const double delta_v_jerk = 0.5 * accel_max * t_jerk;
        const double delta_v_const = accel_max * t_const;
        return 2.0 * delta_v_jerk + delta_v_const;
    } else {
        // Timeout quá ngắn: chỉ có triangular profile
        const double t_half = timeout_sec * 0.5;
        return 0.5 * jerk_max * std::pow(t_half, 2);
    }
}

// Reset all kinematic shaping state (called when disengaging)
void SmartFollowNode::reset_shaping_state() {
    // Reset legacy command history
    last_cmd_forward_ = 0.0;
    last_cmd_left_ = 0.0;
    last_cmd_up_ = 0.0;
    last_cmd_yaw_ = 0.0;
    last_accel_forward_ = 0.0;
    last_accel_left_ = 0.0;
    last_accel_up_ = 0.0;
    last_accel_yaw_ = 0.0;
    
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