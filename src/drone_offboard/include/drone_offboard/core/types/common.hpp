#ifndef DRONE_OFFBOARD_CORE_TYPES_COMMON_HPP_
#define DRONE_OFFBOARD_CORE_TYPES_COMMON_HPP_

/**
 * @file common.hpp
 * @brief Common types, constants, and utilities (NO ROS DEPENDENCY)
 */

#include <cmath>

namespace drone_follow {
namespace core {

// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================
constexpr double R_EARTH = 6378137.0;           // Earth radius (m) WGS84
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// ============================================================================
// CONTROL LOOP CONSTANTS
// ============================================================================
constexpr double CONTROL_LOOP_DT_DEFAULT = 0.02;     // 50Hz default
constexpr double CONTROL_LOOP_DT_MIN = 0.001;
constexpr double CONTROL_LOOP_DT_MAX = 0.5;

// ============================================================================
// FILTER CONSTANTS
// ============================================================================
constexpr double ACCEL_FILTER_ALPHA = 0.15;
constexpr double ACCEL_FILTER_BETA = 0.85;
constexpr double ACCEL_MAGNITUDE_CLAMP = 5.0;        // m/s²
constexpr double YAW_RATE_FILTER_ALPHA = 0.3;
constexpr double YAW_RATE_FILTER_BETA = 0.7;

// ============================================================================
// THRESHOLD CONSTANTS
// ============================================================================
constexpr double GPS_DT_MIN_VALID = 0.01;            // seconds
constexpr double GPS_DT_MAX_VALID = 2.0;             // seconds
constexpr double SPEED_THRESHOLD_HEADING = 0.5;      // m/s
constexpr double SPEED_THRESHOLD_OFFSET = 0.3;       // m/s

// ============================================================================
// SAFETY LIMITS
// ============================================================================
constexpr double MAX_TARGET_VELOCITY = 50.0;         // m/s (~180 km/h)
constexpr double MAX_TARGET_ACCELERATION = 10.0;     // m/s² (~1G)
constexpr double CMD_VEL_FORWARD_MAX = 6.0;         // m/s
constexpr double CMD_VEL_LATERAL_MAX = 5.0;          // m/s
constexpr double CMD_VEL_VERTICAL_MAX = 2.0;         // m/s
constexpr double CMD_YAW_RATE_MAX = 1.5;             // rad/s

// ============================================================================
// GPS MEASUREMENT
// ============================================================================

/**
 * @brief GPS measurement (input to estimator)
 */
struct GPSMeasurement {
    double lat;              // Latitude (degrees)
    double lon;              // Longitude (degrees)
    double alt;              // Altitude MSL (m)
    double timestamp_sec;    // Timestamp (seconds since epoch)
    
    // Optional accuracy info
    double horizontal_accuracy = 999.0;  // m
    double covariance = 999.0;           // m²
    bool has_covariance = false;
};

/**
 * @brief Gimbal measurement (for yaw control)
 */
struct GimbalMeasurement {
    double pan_error_deg;    // Pan angle error (degrees)
    double timestamp_sec;    // Timestamp (seconds since epoch)
};

/**
 * @brief Offset type for follow positioning
 */
enum class OffsetType {
    NED = 0,         // Fixed offset in NED frame
    RELATIVE = 1,    // Offset rotates with target heading
    VELOCITY = 2     // Offset based on velocity direction (behind target)
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_TYPES_COMMON_HPP_
