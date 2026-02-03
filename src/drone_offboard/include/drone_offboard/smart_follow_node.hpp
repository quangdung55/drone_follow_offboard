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
#include <mutex>
#include <stdexcept>

// ArduPilot Control Algorithms
#include "drone_offboard/ap_control.hpp"
#include "drone_offboard/ap_follow.hpp"
#include "drone_offboard/JitterCorrection.h"
#include "ardupilot_msgs/msg/status.hpp"

// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================
constexpr double R_EARTH = 6378137.0;           // Earth radius in meters (WGS84)
constexpr double DEG_TO_RAD = M_PI / 180.0;     // Degrees to radians conversion

// ============================================================================
// CONTROL LOOP CONSTANTS
// ============================================================================
constexpr double CONTROL_LOOP_DT_DEFAULT = 0.02;        // Default dt for 50Hz loop (seconds)
constexpr double CONTROL_LOOP_DT_MIN = 0.001;           // Minimum valid dt (seconds)
constexpr double CONTROL_LOOP_DT_MAX = 0.5;             // Maximum valid dt (seconds)

// ============================================================================
// FILTER CONSTANTS
// ============================================================================
constexpr double ACCEL_FILTER_ALPHA = 0.15;             // Acceleration low-pass filter coefficient
constexpr double ACCEL_MAGNITUDE_CLAMP = 5.0;           // Maximum acceleration magnitude (m/s²)
constexpr double YAW_RATE_FILTER_ALPHA = 0.3;           // Yaw rate low-pass filter coefficient

// ============================================================================
// THRESHOLD CONSTANTS
// ============================================================================
constexpr double GPS_DT_MIN_VALID = 0.01;               // Minimum valid GPS update interval (seconds)
constexpr double GPS_DT_MAX_VALID = 2.0;                // Maximum valid GPS update interval (seconds)
constexpr double SPEED_THRESHOLD_HEADING = 0.5;         // Min speed to calculate heading (m/s)
constexpr double SPEED_THRESHOLD_OFFSET = 0.3;          // Min speed for offset calculation (m/s)
constexpr double GIMBAL_TIMEOUT_SEC = 0.5;              // Gimbal message timeout (seconds)

// ============================================================================
// SAFETY LIMITS
// ============================================================================
constexpr double MAX_TARGET_VELOCITY = 50.0;            // Max valid target velocity (m/s) ~180 km/h
constexpr double MAX_TARGET_ACCELERATION = 10.0;        // Max valid target acceleration (m/s²) ~1G
constexpr double CMD_VEL_FORWARD_MAX = 5.0;             // Max forward velocity command (m/s)
constexpr double CMD_VEL_LATERAL_MAX = 3.0;             // Max lateral velocity command (m/s)
constexpr double CMD_VEL_VERTICAL_MAX = 1.5;            // Max vertical velocity command (m/s)
constexpr double CMD_YAW_RATE_MAX = 0.8;                // Max yaw rate command (rad/s)

// --- Alpha-Beta Filter Class ---
// Bộ lọc thông minh ước lượng cả position và velocity
class AlphaBetaFilter {
public:
    double x = 0.0;  // Estimated position
    double v = 0.0;  // Estimated velocity

    // Standard update with fixed beta
    void update(double measurement, double dt, double alpha, double beta) {
        if (dt <= 0.0) return;
        
        // Prediction step
        double x_pred = x + v * dt;
        
        // Update step
        double residual = measurement - x_pred;
        x = x_pred + alpha * residual;
        v = v + (beta * residual) / dt;
    }
    
    // Adaptive beta update (faster response when residual is large)
    void update_adaptive(double measurement, double dt, double alpha, 
                        double beta_fast, double beta_slow, double residual_threshold) {
        if (dt <= 0.0) return;
        
        // Prediction step
        double x_pred = x + v * dt;
        
        // Update step with adaptive beta
        double residual = measurement - x_pred;
        double abs_r = std::abs(residual);
        double beta_eff = (abs_r > residual_threshold) ? beta_fast : beta_slow;
        
        x = x_pred + alpha * residual;
        v = v + (beta_eff * residual) / dt;
    }
    
    void reset(double measurement) {
        x = measurement;
        v = 0.0;
    }
};

// Target state structure (Enhanced with Eigen)
struct TargetState {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;                    // Altitude MSL from GPS
    double heading_rad = 0.0;            // Movement heading (ROS ENU: 0=East, PI/2=North)
    double yaw_rate = 0.0;               // Target rotation rate (rad/s)
    rclcpp::Time last_update;
    bool valid = false;
};

// Position in Map-ENU frame (CRITICAL!)
// Both drone and target are converted to this frame
struct MapPosition {
    Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();    // Position (East, North, Up) in map frame
    Eigen::Vector3d vel_enu = Eigen::Vector3d::Zero();    // Velocity (East, North, Up)
    Eigen::Vector3d accel_enu = Eigen::Vector3d::Zero();  // Acceleration (East, North, Up)
};

// Navigation command structure
struct NavCommand {
    double vel_forward; // Forward velocity (Body Frame)
    double vel_left;    // Lateral velocity (Body Frame)
    double vel_up;      // Vertical velocity (Body Frame)
    double yaw_rate;    // Rotation rate (rad/s)
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
    void cb_status(const ardupilot_msgs::msg::Status::SharedPtr msg);

    /* --- CORE LOGIC --- */
    void control_loop();
    NavCommand calculate_kinematics();
    void publish_cmd(const NavCommand& cmd);
    void stop_drone();

    /* --- OFFSET TYPES (AP_Follow style) --- */
    enum class OffsetType {
        NED = 0,          // Fixed offset in North-East-Down frame
        RELATIVE = 1,     // Offset relative to target heading
        VELOCITY = 2      // Dynamic offset based on velocity direction (current default)
    };
    
    /* --- HELPERS (PRIVATE) --- */
    void gps_to_map_enu_internal(double lat, double lon, double& e, double& n) const;  // Internal GPS conversion (no lock)
    void apply_kinematic_limits(double& desired_vel, double& last_vel, double& last_acc, 
                                double dt, double max_acc, double max_jerk);
    void apply_sqrt_position_shaping();  // ArduPilot sqrt controller for position
    bool estimate_error_too_large() const;  // AP_Follow style error check
    double calc_max_velocity_change(double accel_max, double jerk_max, double timeout_sec) const;
    void reset_shaping_state();  // Reset all kinematic shaping state
    Eigen::Vector2d calculate_offset_enu(double desired_dist);  // Calculate offset based on mode
    Eigen::Vector2d rotate_vector_2d(const Eigen::Vector2d& vec, double angle_rad) const;  // Rotate 2D vector
    
    /* --- MEMBERS --- */
    // QoS
    rclcpp::SensorDataQoS qos_best_effort_;

    // Subscribers
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr sub_global_origin_;  // EKF Origin
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_drone_gps_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_target_gps_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_gimbal_;
    rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr sub_status_;  // ArduPilot DDS

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
    int current_mode_int_ = 0;

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
    bool param_filter_enable_;       // Enable/disable filtering
    double param_filter_alpha_;      // Position trust (0.0 - 1.0)
    double param_filter_beta_;       // Velocity update rate
    double param_filter_beta_fast_;  // Beta when residual > threshold (fast response)
    double param_filter_beta_slow_;  // Beta when residual <= threshold (smooth)
    double param_filter_residual_threshold_; // Threshold (m) to switch to fast beta
    
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
    
    // Advanced Features - Offset Mode (AP_Follow style)
    OffsetType param_offset_type_;
    Eigen::Vector3d param_offset_ned_;        // Static offset (North, East, Down) in meters
    
    // Advanced Features - Jitter Correction
    bool param_jitter_correction_enable_;     // Enable/disable jitter correction
    uint16_t param_jitter_max_lag_ms_;        // Max transport lag (ms)
    uint16_t param_jitter_convergence_loops_; // Convergence samples
    
    // Thread safety mutex for shared state between callbacks and control loop
    mutable std::mutex state_mutex_;
    
    // Jitter Correction (AP_Follow style)
    std::optional<JitterCorrection> jitter_correction_;  // Initialized in setup_parameters()
    
    // Last estimate for error checking (AP_Follow style)
    Eigen::Vector3d last_estimate_pos_enu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_estimate_vel_enu_ = Eigen::Vector3d::Zero();
    
    // Control timing
    rclcpp::Time last_control_time_;
    
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
};

#endif // SMART_FOLLOW_NODE_HPP_