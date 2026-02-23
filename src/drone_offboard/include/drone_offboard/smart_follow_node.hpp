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

// ROS2 Core (minimal required)
#include <rclcpp/rclcpp.hpp>

// ROS messages (needed for SharedPtr in callbacks - lightweight headers)
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <ardupilot_msgs/msg/status.hpp>

// Core types (need OriginManager complete definition)
#include "drone_offboard/core/estimator/target_estimator.hpp"

// Eigen (need Dense for Zero() and vector operations)
#include <Eigen/Dense>

// STL (only what's needed in header)
#include <mutex>
#include <atomic>
#include <memory>

// Forward declarations - Core components (no ROS dependency)
namespace drone_follow { namespace core {
    class TargetEstimator;
    class FollowSystem;
    class GPSHealthMonitor;
    class SystemHealthMonitor;
    struct SystemHealthReport;
    class OriginManager;  // GPS origin for ENU conversion
}}// ============================================================================
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
// Low-pass filter: y[n] = alpha * x[n] + beta * y[n-1], where beta = (1 - alpha)
constexpr double ACCEL_FILTER_ALPHA = 0.15;             // Acceleration filter: weight for new measurement
constexpr double ACCEL_FILTER_BETA = 0.85;              // Acceleration filter: weight for previous value (1 - alpha)
constexpr double ACCEL_MAGNITUDE_CLAMP = 5.0;           // Maximum acceleration magnitude (m/s²)
constexpr double YAW_RATE_FILTER_ALPHA = 0.3;           // Yaw rate filter: weight for new measurement
constexpr double YAW_RATE_FILTER_BETA = 0.7;            // Yaw rate filter: weight for previous value (1 - alpha)

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
constexpr double CMD_VEL_FORWARD_MAX = 6.0;            // Max forward velocity command (m/s) ~54 km/h
constexpr double CMD_VEL_LATERAL_MAX = 5.0;             // Max lateral velocity command (m/s)
constexpr double CMD_VEL_VERTICAL_MAX = 2.0;            // Max vertical velocity command (m/s)
constexpr double CMD_YAW_RATE_MAX = 1.5;                // Max yaw rate command (rad/s)
constexpr double MAX_ALTITUDE_AGL = 120.0;              // Maximum altitude above ground (m)
constexpr double MAX_ALTITUDE_MSL = 800.0;              // Maximum altitude above sea level (m) - Increased for high-altitude sites
constexpr double GPS_COVARIANCE_WARN_THRESHOLD = 10.0;  // GPS accuracy warning (m²)
constexpr double GPS_COVARIANCE_MAX_THRESHOLD = 25.0;   // GPS accuracy failure (m²)
constexpr double GPS_FROZEN_TIMEOUT = 2.0;              // GPS stream frozen timeout (s)
// ============================================================================
// STATE STRUCTURES (NO LOGIC, JUST STATE)
// ============================================================================

// ============================================================================
// GPS HEALTH MONITORING
// ============================================================================

// Simple GPS health state (NO logic methods)
struct GPSHealth {
    double horizontal_accuracy = 999.0;
    double covariance = 999.0;
    rclcpp::Time last_update;
    bool frozen_detected = false;
    bool has_covariance = false;
    int satellites = 0;
};

/**
 * @brief Target state in GPS coordinates and movement parameters
 */
struct TargetState {
    // GPS Position
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;                    // Altitude MSL from GPS
    
    // Map-ENU Position (meters from origin)
    Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();    // Position (East, North, Up)
    Eigen::Vector3d vel_enu = Eigen::Vector3d::Zero();    // Velocity (East, North, Up)
    Eigen::Vector3d accel_enu = Eigen::Vector3d::Zero();  // Acceleration (East, North, Up)
    
    // Movement
    double heading_rad = 0.0;            // Movement heading (ROS ENU: 0=East, PI/2=North)
    double yaw_rate = 0.0;               // Target rotation rate (rad/s)
    
    // Timing and validity
    rclcpp::Time last_update;
    bool valid = false;
    
    /**
     * @brief Reset target state to invalid
     */
    void reset() {
        valid = false;
        heading_rad = 0.0;
        yaw_rate = 0.0;
        pos_enu.setZero();
        vel_enu.setZero();
        accel_enu.setZero();
    }
};

/**
 * @brief Drone state with GPS and pose information
 */
struct DroneState {
    // GPS Position (WGS84)
    double lat = 0.0;
    double lon = 0.0;
    double alt_msl = 0.0;          // Mean Sea Level altitude
    double alt_rel = 0.0;           // Relative to home
    
    // Map-ENU Position (meters from origin)
    Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();
    
    // Orientation
    double yaw = 0.0;               // radians, ENU frame
    
    // Timestamps
    rclcpp::Time last_gps_update;
    rclcpp::Time last_pose_update;
    
    // Validity flags
    bool gps_valid = false;
    bool pose_valid = false;
    
    /**
     * @brief Check if drone state is ready for control
     */
    bool is_ready() const {
        return gps_valid && pose_valid;
    }
    
    /**
     * @brief Reset all state
     */
    void reset() {
        gps_valid = false;
        pose_valid = false;
        pos_enu.setZero();
        yaw = 0.0;
    }
};

/**
 * @brief Navigation command in body frame
 */
struct NavCommand {
    double vel_forward; // Forward velocity (Body Frame, m/s)
    double vel_left;    // Lateral velocity (Body Frame, m/s)
    double vel_up;      // Vertical velocity (Body Frame, m/s)
    double yaw_rate;    // Rotation rate (rad/s)
};

// ============================================================================
// NODE CLASS
// ============================================================================

// ALL VALIDATION/GIMBAL/TRANSITION LOGIC MOVED TO CORE LAYER
// ROS layer only handles lifecycle and state management

class SmartFollowNode : public rclcpp::Node {
public:
    SmartFollowNode();
    ~SmartFollowNode();  // Declared here, defined in .cpp (for unique_ptr of forward-declared types)
    
    // Helpers (public for testing)
    void gps_to_map_enu(double lat, double lon, double& e, double& n) const;  // Convert GPS → Map-ENU

private:
    /* --- SETUP FUNCTIONS --- */
    void setup_parameters();
    void initialize_core_components();  // Initialize core algorithm components (NO ROS dependency)
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
    void publish_cmd(const NavCommand& cmd);
    void stop_drone();
    
    /* --- HELPERS (PRIVATE) - Minimal, all logic in core --- */
    void publish_diagnostics();
    
private:
    /* --- ROS COMMUNICATION --- */
    rclcpp::SensorDataQoS qos_best_effort_;
    
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr sub_global_origin_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_drone_gps_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_target_gps_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_gimbal_;
    rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr sub_status_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diagnostics_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    /* --- STATE MANAGEMENT (THREAD-SAFE) --- */
    drone_follow::core::OriginManager origin_manager_;
    DroneState drone_state_;
    TargetState target_state_;
    mutable std::mutex state_mutex_;  // Protects drone_state_, target_state_
    
    /* --- CACHED STATE (accessed in control loop) --- */
    double drone_yaw_ = 0.0;
    double drone_relative_alt_ = 0.0;
    double gimbal_pan_error_ = 0.0;
    bool pose_ready_ = false;
    rclcpp::Time last_gimbal_msg_;
    
    /* --- FLIGHT SAFETY (ATOMIC) --- */
    std::atomic<bool> is_guided_{false};
    std::atomic<bool> is_armed_{false};
    std::atomic<int> current_mode_int_{0};
    
    /* --- TIMING --- */
    rclcpp::Time last_control_time_;
    rclcpp::Time last_target_update_;
    rclcpp::Time last_gimbal_update_;


    /* --- CORE UNIFIED SYSTEM (NO ROS DEPENDENCY!) --- */
    std::unique_ptr<drone_follow::core::TargetEstimator> target_estimator_;
    std::unique_ptr<drone_follow::core::FollowSystem> follow_system_;
    std::unique_ptr<drone_follow::core::GPSHealthMonitor> gps_health_monitor_;
    std::unique_ptr<drone_follow::core::SystemHealthMonitor> system_health_monitor_;
    
    /* --- GPS HEALTH MONITORING --- */
    GPSHealth drone_gps_health_;
    GPSHealth target_gps_health_;

    /* --- PARAMETERS --- */
    // Basic
    double param_follow_dist_;
    double param_follow_height_;
    double param_pred_time_;
    double param_timeout_;
    double param_kp_pos_;
    double param_kp_vel_z_;
    double param_kp_yaw_;
    double param_deadzone_;
    
    // Kinematic Constraints
    double param_accel_max_ne_;
    double param_jerk_max_ne_;
    double param_accel_max_d_;
    double param_jerk_max_d_;
    double param_accel_max_h_;
    double param_jerk_max_h_;
    
    // Feed Forward
    double param_ff_lateral_gain_;
    double param_ff_centripetal_gain_;

    // Filter
    bool param_filter_enable_;
    double param_filter_alpha_;
    double param_filter_beta_;
    double param_filter_beta_fast_;
    double param_filter_beta_slow_;
    double param_filter_residual_threshold_;
    
    // Advanced Features
    bool param_adaptive_dist_enable_;
    double param_follow_dist_min_;
    double param_follow_dist_max_;
    double param_dist_speed_gain_;
    bool param_heading_blend_enable_;
    double param_heading_blend_weight_;
    double param_heading_follow_kp_;
    bool param_terrain_follow_enable_;
    drone_follow::core::OffsetType param_offset_type_;
    Eigen::Vector3d param_offset_ned_;
    bool param_jitter_correction_enable_;
    uint16_t param_jitter_max_lag_ms_;
    uint16_t param_jitter_convergence_loops_;

};

#endif // SMART_FOLLOW_NODE_HPP_