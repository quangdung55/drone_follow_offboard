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
#include <diagnostic_msgs/msg/diagnostic_status.hpp>  // Diagnostics
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <optional>
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

// ============================================================================
// STATE STRUCTURES
// ============================================================================

// ============================================================================
// GPS HEALTH MONITORING
// ============================================================================

/**
 * @brief GPS health status with accuracy and watchdog tracking
 */
struct GPSHealth {
    double horizontal_accuracy = 999.0;  // Horizontal position accuracy (m)
    double covariance = 999.0;            // Position covariance (m²)
    rclcpp::Time last_update;
    bool frozen_detected = false;
    bool has_covariance = false;          // Whether sensor provides covariance
    int satellites = 0;
    
    bool is_healthy() const {
        // If no covariance data, rely only on frozen detection
        if (!has_covariance) {
            return !frozen_detected;
        }
        // If has covariance, check threshold
        return covariance < GPS_COVARIANCE_MAX_THRESHOLD && !frozen_detected;
    }
    
    bool needs_warning() const {
        // Only warn if we have covariance data
        return has_covariance && covariance > GPS_COVARIANCE_WARN_THRESHOLD;
    }
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
// THREAD-SAFE ORIGIN MANAGER
// ============================================================================

/**
 * @brief Thread-safe manager for EKF global origin
 * 
 * Ensures origin is set only once and provides lock-free atomic checks.
 * Critical for preventing race conditions during initialization.
 */
class OriginManager {
public:
    OriginManager() : ready_(false) {}
    
    /**
     * @brief Set origin (thread-safe, only once)
     * @return true if origin was set, false if already set
     */
    bool set_origin(double lat, double lon, double alt) {
        bool expected = false;
        // Atomic compare-exchange ensures only one thread sets origin
        if (ready_.compare_exchange_strong(expected, true)) {
            std::lock_guard<std::mutex> lock(mutex_);
            lat_ = lat;
            lon_ = lon;
            alt_ = alt;
            return true;
        }
        return false; // Already set
    }
    
    /**
     * @brief Check if origin is ready (lock-free)
     */
    bool is_ready() const {
        return ready_.load(std::memory_order_acquire);
    }
    
    /**
     * @brief Get origin values (thread-safe)
     * @return true if origin is available
     */
    bool get_origin(double& lat, double& lon, double& alt) const {
        if (!is_ready()) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        lat = lat_;
        lon = lon_;
        alt = alt_;
        return true;
    }
    
    /**
     * @brief Convert GPS to Map-ENU (thread-safe)
     */
    bool gps_to_enu(double lat, double lon, double& east, double& north) const {
        double origin_lat, origin_lon, origin_alt;
        if (!get_origin(origin_lat, origin_lon, origin_alt)) {
            return false;
        }
        
        double d_lat = (lat - origin_lat) * DEG_TO_RAD;
        double d_lon = (lon - origin_lon) * DEG_TO_RAD;
        double lat0_rad = origin_lat * DEG_TO_RAD;
        
        north = d_lat * R_EARTH;
        east = d_lon * R_EARTH * std::cos(lat0_rad);
        
        return true;
    }
    
private:
    std::atomic<bool> ready_;
    mutable std::mutex mutex_;
    double lat_ = 0.0;
    double lon_ = 0.0;
    double alt_ = 0.0;
};

// ============================================================================
// DT VALIDATOR WITH STATE MANAGEMENT
// ============================================================================

/**
 * @brief Validates time deltas and triggers state resets on invalid timing
 * 
 * Prevents control instability from timing glitches by:
 * - Rejecting too-small dt (division issues)
 * - Detecting timeouts (sensor loss)
 * - Providing safe fallback dt values
 */
class DtValidator {
public:
    struct ValidationResult {
        double dt;
        bool valid;
        bool should_reset;
    };
    
    DtValidator(double min_dt, double max_dt, double default_dt)
        : min_dt_(min_dt), max_dt_(max_dt), default_dt_(default_dt),
          last_valid_time_(0.0), valid_count_(0) {}
    
    /**
     * @brief Validate time delta
     * @param current_time Current timestamp in seconds
     * @return Validation result with dt and reset flag
     */
    ValidationResult validate(double current_time) {
        ValidationResult result;
        
        if (last_valid_time_ == 0.0) {
            // First call - initialize
            last_valid_time_ = current_time;
            result.dt = default_dt_;
            result.valid = false;
            result.should_reset = false;  // Changed: Don't reset on first call, just skip
            return result;
        }
        
        double dt = current_time - last_valid_time_;
        
        // Special case: duplicate timestamp (dt = 0)
        if (dt == 0.0) {
            result.dt = default_dt_;
            result.valid = false;
            result.should_reset = false;  // Don't reset, just skip this update
            return result;
        }
        
        if (dt < min_dt_ || dt > max_dt_) {
            // Invalid dt detected
            result.dt = default_dt_;
            result.valid = false;
            result.should_reset = (dt > max_dt_); // Reset ONLY if timeout
            
            // Don't update last_valid_time_ - wait for valid update
            return result;
        }
        
        // Valid dt
        last_valid_time_ = current_time;
        result.dt = dt;
        result.valid = true;
        result.should_reset = false;
        valid_count_++;
        
        return result;
    }
    
    void reset() {
        last_valid_time_ = 0.0;
        valid_count_ = 0;
    }
    
    int get_valid_count() const { return valid_count_; }
    double get_last_valid_time() const { return last_valid_time_; }
    
private:
    double min_dt_;
    double max_dt_;
    double default_dt_;
    double last_valid_time_;
    int valid_count_;
};

// ============================================================================
// SAFE GIMBAL TRACKER
// ============================================================================

/**
 * @brief Thread-safe gimbal error tracking with timeout
 * 
 * Prevents undefined behavior from using gimbal data before first message
 */
class GimbalTracker {
public:
    GimbalTracker(double timeout_sec) 
        : timeout_sec_(timeout_sec), 
          pan_error_(0.0), 
          first_message_received_(false) {}
    
    void update(double pan_error, const rclcpp::Time& timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        pan_error_ = pan_error;
        last_update_ = timestamp;
        first_message_received_ = true;
    }
    
    /**
     * @brief Get gimbal pan error if valid and recent
     * @return true if gimbal data is valid and recent
     */
    bool get_error(double& pan_error, const rclcpp::Time& now) const {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!first_message_received_) {
            return false;
        }
        
        double time_since_update = (now - last_update_).seconds();
        if (time_since_update > timeout_sec_) {
            return false;
        }
        
        pan_error = pan_error_;
        return true;
    }
    
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        first_message_received_ = false;
        pan_error_ = 0.0;
    }
    
private:
    mutable std::mutex mutex_;
    double timeout_sec_;
    double pan_error_;
    rclcpp::Time last_update_;
    bool first_message_received_;
};

// ============================================================================
// VELOCITY SANITY CHECKER
// ============================================================================

/**
 * @brief Validates velocity commands before publishing
 * 
 * Catches NaN, infinity, and excessive velocities that could damage hardware
 */
class VelocitySanityChecker {
public:
    struct Limits {
        double max_forward;
        double max_lateral;
        double max_vertical;
        double max_yaw_rate;
    };
    
    struct CheckResult {
        bool passed;
        std::string reason;
    };
    
    VelocitySanityChecker(const Limits& limits) : limits_(limits) {}
    
    /**
     * @brief Check if velocity command is safe
     */
    CheckResult check(double vx, double vy, double vz, double yaw_rate) const {
        CheckResult result{true, ""};
        
        if (!std::isfinite(vx) || !std::isfinite(vy) || 
            !std::isfinite(vz) || !std::isfinite(yaw_rate)) {
            result.passed = false;
            result.reason = "Non-finite velocity command";
            return result;
        }
        
        if (std::abs(vx) > limits_.max_forward) {
            result.passed = false;
            result.reason = "Forward velocity exceeds limit";
            return result;
        }
        
        if (std::abs(vy) > limits_.max_lateral) {
            result.passed = false;
            result.reason = "Lateral velocity exceeds limit";
            return result;
        }
        
        if (std::abs(vz) > limits_.max_vertical) {
            result.passed = false;
            result.reason = "Vertical velocity exceeds limit";
            return result;
        }
        
        if (std::abs(yaw_rate) > limits_.max_yaw_rate) {
            result.passed = false;
            result.reason = "Yaw rate exceeds limit";
            return result;
        }
        
        return result;
    }
    
    /**
     * @brief Clamp velocities to safe limits
     */
    void clamp(double& vx, double& vy, double& vz, double& yaw_rate) const {
        vx = std::clamp(vx, -limits_.max_forward, limits_.max_forward);
        vy = std::clamp(vy, -limits_.max_lateral, limits_.max_lateral);
        vz = std::clamp(vz, -limits_.max_vertical, limits_.max_vertical);
        yaw_rate = std::clamp(yaw_rate, -limits_.max_yaw_rate, limits_.max_yaw_rate);
    }
    
private:
    Limits limits_;
};

// ============================================================================
// SMOOTH MODE TRANSITION MANAGER
// ============================================================================

/**
 * @brief Manages smooth transitions between terrain follow modes
 * 
 * Uses S-curve blending to prevent sudden altitude jumps when switching
 * between terrain-relative and fixed-altitude modes
 */
class TerrainFollowTransition {
public:
    TerrainFollowTransition(double transition_time_sec = 2.0)
        : transition_time_(transition_time_sec),
          transitioning_(false),
          blend_factor_(0.0) {}
    
    void set_mode(bool terrain_follow, const rclcpp::Time& now) {
        if (terrain_follow != target_mode_) {
            // Mode change initiated
            target_mode_ = terrain_follow;
            transitioning_ = true;
            transition_start_ = now;
            start_blend_factor_ = blend_factor_;
        }
    }
    
    /**
     * @brief Get blended altitude command during transition
     * @param terrain_alt Altitude in terrain-follow mode
     * @param fixed_alt Altitude in fixed-height mode
     * @return Blended altitude command
     */
    double get_altitude_command(double terrain_alt, double fixed_alt, 
                                const rclcpp::Time& now) {
        if (transitioning_) {
            double elapsed = (now - transition_start_).seconds();
            double t = std::min(elapsed / transition_time_, 1.0);
            
            // Smooth S-curve transition (3t² - 2t³)
            double s = 3.0 * t * t - 2.0 * t * t * t;
            
            if (target_mode_) {
                // Transitioning TO terrain follow
                blend_factor_ = start_blend_factor_ + s * (1.0 - start_blend_factor_);
            } else {
                // Transitioning FROM terrain follow
                blend_factor_ = start_blend_factor_ - s * start_blend_factor_;
            }
            
            if (t >= 1.0) {
                transitioning_ = false;
                blend_factor_ = target_mode_ ? 1.0 : 0.0;
            }
        }
        
        // Blend between modes
        return blend_factor_ * terrain_alt + (1.0 - blend_factor_) * fixed_alt;
    }
    
    bool is_transitioning() const { return transitioning_; }
    
private:
    double transition_time_;
    bool transitioning_;
    bool target_mode_ = false;
    double blend_factor_;
    double start_blend_factor_;
    rclcpp::Time transition_start_;
};


class SmartFollowNode : public rclcpp::Node {
public:
    SmartFollowNode();
    ~SmartFollowNode() = default;
    
    // Helpers (public for testing)
    void gps_to_map_enu(double lat, double lon, double& e, double& n) const;  // Convert GPS → Map-ENU

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
    NavCommand calculate_kinematics(const DroneState& drone, const TargetState& target, double dt);
    void publish_cmd(const NavCommand& cmd);
    void stop_drone();

    /* --- OFFSET TYPES (AP_Follow style) --- */
    enum class OffsetType {
        NED = 0,          // Fixed offset in North-East-Down frame
        RELATIVE = 1,     // Offset relative to target heading
        VELOCITY = 2      // Dynamic offset based on velocity direction (current default)
    };
    
    /* --- HELPERS (PRIVATE) --- */
    void apply_kinematic_limits(double& desired_vel, double& last_vel, double& last_acc, 
                                double dt, double max_acc, double max_jerk);
    void apply_sqrt_position_shaping();  // ArduPilot sqrt controller for position
    bool estimate_error_too_large(const TargetState& target) const;
    double calc_max_velocity_change(double accel_max, double jerk_max, double timeout_sec) const;
    void reset_shaping_state();  // Reset all kinematic shaping state
    bool check_altitude_safety(const DroneState& drone) const;
    void emergency_descent();
    void update_gps_health(const sensor_msgs::msg::NavSatFix::SharedPtr msg, GPSHealth& health);
    void check_gps_frozen(GPSHealth& health, const rclcpp::Time& now);
    void publish_diagnostics();
    Eigen::Vector2d calculate_offset_enu(const TargetState& target, double desired_dist);
    Eigen::Vector2d rotate_vector_2d(const Eigen::Vector2d& vec, double angle_rad) const;
    double calculate_yaw_rate(const DroneState& drone, const TargetState& target, double dt);
    void gps_to_map_enu_internal(double lat, double lon, double& e, double& n) const;
    
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
    OriginManager origin_manager_;
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
    
    /* --- TIMING AND VALIDATION --- */
    GimbalTracker gimbal_tracker_{GIMBAL_TIMEOUT_SEC};
    DtValidator target_dt_validator_{GPS_DT_MIN_VALID, GPS_DT_MAX_VALID, 0.1};
    DtValidator control_dt_validator_{CONTROL_LOOP_DT_MIN, CONTROL_LOOP_DT_MAX, CONTROL_LOOP_DT_DEFAULT};
    VelocitySanityChecker velocity_checker_{{
        CMD_VEL_FORWARD_MAX,
        CMD_VEL_LATERAL_MAX,
        CMD_VEL_VERTICAL_MAX,
        CMD_YAW_RATE_MAX
    }};
    TerrainFollowTransition terrain_transition_{2.0};
    rclcpp::Time last_control_time_;

    /* --- FILTERS --- */
    AlphaBetaFilter filter_vel_e_;   // East velocity filter
    AlphaBetaFilter filter_vel_n_;   // North velocity filter
    AlphaBetaFilter filter_alt_;     // Altitude filter
    std::optional<JitterCorrection> jitter_correction_;  // GPS timestamp correction

    /* --- KINEMATIC SHAPING STATE --- */
    Eigen::Vector2d shaped_vel_xy_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d shaped_accel_xy_ = Eigen::Vector2d::Zero();
    double shaped_vel_z_ = 0.0;
    double shaped_accel_z_ = 0.0;
    double shaped_yaw_rate_ = 0.0;
    double shaped_yaw_accel_ = 0.0;
    ap_follow::FollowEstimator follow_estimator_;
    Eigen::Vector3d last_estimate_pos_enu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_estimate_vel_enu_ = Eigen::Vector3d::Zero();
    double filtered_desired_dist_ = 5.0;
    
    /* --- GPS HEALTH MONITORING --- */
    GPSHealth drone_gps_health_;
    GPSHealth target_gps_health_;
    std::atomic<bool> emergency_descent_active_{false};

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
    OffsetType param_offset_type_;
    Eigen::Vector3d param_offset_ned_;
    bool param_jitter_correction_enable_;
    uint16_t param_jitter_max_lag_ms_;
    uint16_t param_jitter_convergence_loops_;

};

#endif // SMART_FOLLOW_NODE_HPP_