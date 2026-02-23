#ifndef DRONE_OFFBOARD_CORE_TYPES_TARGET_STATE_HPP_
#define DRONE_OFFBOARD_CORE_TYPES_TARGET_STATE_HPP_

/**
 * @file target_state.hpp
 * @brief Pure C++ target state types (NO ROS DEPENDENCY)
 */

#include <Eigen/Dense>

namespace drone_follow {
namespace core {

/**
 * @brief Target state with GPS position and estimated motion
 * 
 * Contains:
 * - GPS position (WGS84)
 * - Map-ENU position/velocity/acceleration
 * - Movement heading and yaw rate
 */
struct TargetState {
    // GPS Position
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;                    // Altitude MSL from GPS (m)
    
    // Map-ENU Position/Velocity/Acceleration (meters, m/s, m/s²)
    Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();    // (East, North, Up)
    Eigen::Vector3d vel_enu = Eigen::Vector3d::Zero();    // (East, North, Up)
    Eigen::Vector3d accel_enu = Eigen::Vector3d::Zero();  // (East, North, Up)
    
    // Movement
    double heading_rad = 0.0;            // Movement heading (ENU: 0=East, π/2=North)
    double yaw_rate = 0.0;               // Target rotation rate (rad/s)
    
    // Timing and validity
    double last_update_sec = 0.0;        // Timestamp (seconds since epoch)
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
        last_update_sec = 0.0;
    }
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_TYPES_TARGET_STATE_HPP_
