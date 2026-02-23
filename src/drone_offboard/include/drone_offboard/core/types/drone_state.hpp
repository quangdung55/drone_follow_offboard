#ifndef DRONE_OFFBOARD_CORE_TYPES_DRONE_STATE_HPP_
#define DRONE_OFFBOARD_CORE_TYPES_DRONE_STATE_HPP_

/**
 * @file drone_state.hpp
 * @brief Pure C++ drone state types (NO ROS DEPENDENCY)
 * 
 * Can be used in:
 * - ROS2 nodes
 * - PX4 modules
 * - SITL
 * - Unit tests
 */

#include <Eigen/Dense>

namespace drone_follow {
namespace core {

/**
 * @brief Drone state with GPS and pose information
 * 
 * Pure value type - no methods, no ROS dependencies
 */
struct DroneState {
    // GPS Position (WGS84)
    double lat = 0.0;
    double lon = 0.0;
    double alt_msl = 0.0;          // Mean Sea Level altitude (m)
    double alt_rel = 0.0;          // Relative to home (m)
    
    // Map-ENU Position (meters from origin)
    Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();
    
    // Orientation
    double yaw = 0.0;              // radians, ENU frame
    
    // Validity flags
    bool gps_valid = false;
    bool pose_valid = false;
    
    // Measurement timestamps (seconds since epoch)
    double last_gps_update_sec = 0.0;
    double last_pose_update_sec = 0.0;
    
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
        last_gps_update_sec = 0.0;
        last_pose_update_sec = 0.0;
    }
};

} // namespace core
} // namespace drone_follow

#endif // DRONE_OFFBOARD_CORE_TYPES_DRONE_STATE_HPP_
