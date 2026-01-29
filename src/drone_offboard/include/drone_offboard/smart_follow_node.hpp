#ifndef SMART_FOLLOW_NODE_HPP_
#define SMART_FOLLOW_NODE_HPP_

/**
 * SMART FOLLOW NODE FOR ARDUPILOT (ROS 2 NATIVE DDS)
 * 
 * Features:
 * - GPS-based Target Following with Prediction
 * - Alpha-Beta Filtering for Noise Reduction
 * - Kinematic Input Shaping (Jerk/Accel Limits)
 * - Lateral Feed-Forward (Anti-Understeer)
 * - Centripetal Acceleration Compensation
 * - Terrain Following (Target Altitude Tracking)
 * - ArduPilot DDS Native (No MAVROS)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>  // ArduPilot DDS: Global Origin
#include <std_msgs/msg/string.hpp>       // ArduPilot DDS: Flight Mode
#include <std_msgs/msg/bool.hpp>         // ArduPilot DDS: Armed Status
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <algorithm>

// ArduPilot Control Algorithms
#include "drone_offboard/ap_control.hpp"
#include "drone_offboard/ap_follow.hpp"

// Hằng số vật lý
constexpr double R_EARTH = 6378137.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;

// --- Alpha-Beta Filter Class ---
// Bộ lọc thông minh ước lượng cả position và velocity
class AlphaBetaFilter {
public:
    double x = 0.0;  // Estimated position
    double v = 0.0;  // Estimated velocity

    void update(double measurement, double dt, double alpha, double beta) {
        if (dt <= 0.0) return;
        
        // Prediction step
        double x_pred = x + v * dt;
        
        // Update step
        double residual = measurement - x_pred;
        x = x_pred + alpha * residual;
        v = v + (beta * residual) / dt;
    }
    
    void reset(double measurement) {
        x = measurement;
        v = 0.0;
    }
};

// Struct lưu trạng thái mục tiêu (Enhanced với Eigen)
struct TargetState {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;                    // Altitude MSL từ GPS
    double heading_rad = 0.0;            // Hướng di chuyển (ROS ENU: 0=East, PI/2=North)
    double yaw_rate = 0.0;               // Tốc độ quay của mục tiêu (rad/s)
    rclcpp::Time last_update;
    bool valid = false;
};

// Struct lưu vị trí trong Map-ENU frame (QUAN TRỌNG!)
// Cả drone và target đều được convert về frame này
struct MapPosition {
    Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();    // Position (East, North, Up) trong map frame
    Eigen::Vector3d vel_enu = Eigen::Vector3d::Zero();    // Velocity (East, North, Up)
    Eigen::Vector3d accel_enu = Eigen::Vector3d::Zero();  // Acceleration (East, North, Up)
};

// Struct lưu lệnh điều khiển sau tính toán
struct NavCommand {
    double vel_forward; // Vận tốc tới (Body Frame)
    double vel_left;    // Vận tốc ngang (Body Frame)
    double vel_up;      // Vận tốc lên (Body Frame)
    double yaw_rate;    // Tốc độ xoay (rad/s)
};

class SmartFollowNode : public rclcpp::Node {
public:
    SmartFollowNode();
    ~SmartFollowNode() = default;
    
    // Helpers (public for testing)
    void gps_to_enu_delta(double lat_ref, double lon_ref, double lat_target, double lon_target, double& d_north, double& d_east);
    void gps_to_map_enu(double lat, double lon, double& e, double& n) const;  // Convert GPS → Map-ENU
    double shape_velocity(double desired_vel, double current_vel, double dt, double accel_max);

private:
    /* --- SETUP FUNCTIONS --- */
    void setup_parameters();
    void setup_qos();
    void setup_pub_sub();

    /* --- CALLBACKS --- */
    void cb_global_origin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg);  // EKF Origin
    void cb_drone_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void cb_drone_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void cb_target_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void cb_gimbal_angle(const geometry_msgs::msg::Point::SharedPtr msg);
    void cb_flight_mode(const std_msgs::msg::String::SharedPtr msg);   // ArduPilot DDS
    void cb_armed_status(const std_msgs::msg::Bool::SharedPtr msg);    // ArduPilot DDS

    /* --- CORE LOGIC --- */
    void control_loop();
    NavCommand calculate_kinematics();
    void publish_cmd(const NavCommand& cmd);
    void stop_drone();

    /* --- HELPERS (PRIVATE) --- */
    void apply_kinematic_limits(double& desired_vel, double& last_vel, double& last_acc, 
                                double dt, double max_acc, double max_jerk);
    void apply_sqrt_position_shaping();  // ArduPilot sqrt controller for position
    bool estimate_error_too_large() const;  // AP_Follow style error check
    double calc_max_velocity_change(double accel_max, double jerk_max, double timeout_sec) const;
    void reset_shaping_state();  // Reset all kinematic shaping state
    
    /* --- MEMBERS --- */
    // QoS
    rclcpp::SensorDataQoS qos_best_effort_;

    // Subscribers
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr sub_global_origin_;  // EKF Origin
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_drone_gps_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_target_gps_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_gimbal_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_flight_mode_;  // ArduPilot DDS
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_armed_;          // ArduPilot DDS

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State Variables - EKF Global Origin (Fixed Reference)
    double origin_lat_ = 0, origin_lon_ = 0, origin_alt_ = 0;
    bool origin_ready_ = false;      // QUAN TRỌNG: Không control khi chưa có origin
    
    // State Variables - Map Frame Positions (Core Architecture!)
    // Drone & Target đều được convert về cùng 1 frame (Map-ENU)
    MapPosition drone_map_;          // Drone position/velocity trong Map-ENU
    MapPosition target_map_;         // Target position/velocity trong Map-ENU
    
    // State Variables - Drone (raw GPS for conversion)
    TargetState target_;             // Target GPS raw data + metadata
    double drone_lat_ = 0, drone_lon_ = 0, drone_alt_ = 0;
    double drone_relative_alt_ = 0;  // Altitude so với Home (từ pose)
    double drone_yaw_ = 0;           // Heading hiện tại của Drone (ROS ENU)
    double gimbal_pan_error_ = 0.0;  // Góc lệch Gimbal (độ)
    bool drone_ready_ = false;
    bool pose_ready_ = false;
    rclcpp::Time last_gimbal_msg_;

    // Flight Safety (ArduPilot DDS Native)
    bool is_guided_ = false;         // GUIDED mode active
    bool is_armed_ = false;          // Vehicle armed
    std::string current_flight_mode_ = "";

    // Alpha-Beta Filters (thay thế Low-pass đơn giản)
    AlphaBetaFilter filter_vel_n_;   // Velocity North filter
    AlphaBetaFilter filter_vel_e_;   // Velocity East filter
    AlphaBetaFilter filter_alt_;     // Altitude filter (Terrain Following)

    // Parameters Cache - Basic
    double param_follow_dist_;
    double param_follow_height_;     // Đổi từ follow_alt -> follow_height (relative to target)
    double param_pred_time_;
    double param_timeout_;           // AP_Follow style configurable timeout
    double param_kp_pos_;
    double param_kp_vel_z_;          // Thêm P gain riêng cho Z
    double param_kp_yaw_;
    double param_deadzone_;
    
    // Parameters - Kinematic Constraints (AP_Follow style - tách riêng cho từng trục)
    double param_accel_max_ne_;      // FOLL_ACCEL_NE: Horizontal accel limit (m/s²)
    double param_jerk_max_ne_;       // FOLL_JERK_NE: Horizontal jerk limit (m/s³)
    double param_accel_max_d_;       // FOLL_ACCEL_D: Vertical accel limit (m/s²)
    double param_jerk_max_d_;        // FOLL_JERK_D: Vertical jerk limit (m/s³)
    double param_accel_max_h_;       // FOLL_ACCEL_H: Heading accel limit (rad/s²)
    double param_jerk_max_h_;        // FOLL_JERK_H: Heading jerk limit (rad/s³)
    
    // Parameters - Feed Forward
    double param_ff_lateral_gain_;   // Lateral feed-forward gain (0.0 - 1.0)
    double param_ff_centripetal_gain_; // Centripetal compensation gain

    // Parameters - Alpha-Beta Filter
    double param_filter_alpha_;      // Position trust (0.0 - 1.0)
    double param_filter_beta_;       // Velocity update rate
    
    // Advanced Features - Adaptive Distance
    bool param_adaptive_dist_enable_;
    double param_follow_dist_min_;
    double param_follow_dist_max_;
    double param_dist_speed_gain_;
    
    // Advanced Features - Heading Blending
    bool param_heading_blend_enable_;
    double param_heading_blend_weight_;
    double param_heading_follow_kp_;

    // Advanced Features - Terrain Following
    bool param_terrain_follow_enable_;
    
    // Command History (for velocity shaping)
    double last_cmd_forward_ = 0.0;
    double last_cmd_left_ = 0.0;
    double last_cmd_up_ = 0.0;
    double last_cmd_yaw_ = 0.0;
    rclcpp::Time last_control_time_;
    
    // Acceleration History (for jerk limiting - 2-stage shaping)
    double last_accel_forward_ = 0.0;
    double last_accel_left_ = 0.0;
    double last_accel_up_ = 0.0;
    double last_accel_yaw_ = 0.0;
    
    // Smoothed adaptive distance (low-pass filtered)
    double filtered_desired_dist_ = 5.0;
    
    // =========================================================================
    // ArduPilot Kinematic Shaping State (NEW - replaces simple jerk limiting)
    // =========================================================================
    
    // Position shaping state (ENU frame) - for sqrt controller
    Eigen::Vector2d shaped_vel_xy_ = Eigen::Vector2d::Zero();      // Shaped horizontal velocity
    Eigen::Vector2d shaped_accel_xy_ = Eigen::Vector2d::Zero();    // Shaped horizontal acceleration
    double shaped_vel_z_ = 0.0;                                     // Shaped vertical velocity
    double shaped_accel_z_ = 0.0;                                   // Shaped vertical acceleration
    
    // Heading shaping state
    double shaped_yaw_rate_ = 0.0;                                  // Shaped yaw rate (rad/s)
    double shaped_yaw_accel_ = 0.0;                                 // Shaped yaw acceleration (rad/s²)
    
    // Follow Estimator (ArduPilot AP_Follow style)
    ap_follow::FollowEstimator follow_estimator_;
    bool use_ap_estimator_ = true;  // Enable/disable AP_Follow estimator
};

#endif // SMART_FOLLOW_NODE_HPP_