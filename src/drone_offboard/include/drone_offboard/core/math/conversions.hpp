#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace drone_follow {
namespace core {

/**
 * @brief Math conversion utilities (NO ROS dependency)
 */
class Conversions {
public:
    /**
     * @brief Convert quaternion to yaw angle (ENU frame)
     * @param w, x, y, z Quaternion components
     * @return Yaw angle in radians
     */
    static double quaternion_to_yaw(double w, double x, double y, double z) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    
    /**
     * @brief Convert quaternion to yaw (Eigen version)
     */
    static double quaternion_to_yaw(const Eigen::Quaterniond& q) {
        return quaternion_to_yaw(q.w(), q.x(), q.y(), q.z());
    }
    
    /**
     * @brief GPS to Local ENU coordinates
     * @param lat, lon Current GPS position (degrees)
     * @param origin_lat, origin_lon Origin GPS position (degrees)
     * @param east, north Output ENU coordinates (meters)
     */
    static void gps_to_enu(double lat, double lon, 
                           double origin_lat, double origin_lon,
                           double& east, double& north) {
        constexpr double R_EARTH = 6378137.0;
        constexpr double DEG_TO_RAD = M_PI / 180.0;
        
        double d_lat = (lat - origin_lat) * DEG_TO_RAD;
        double d_lon = (lon - origin_lon) * DEG_TO_RAD;
        double lat0_rad = origin_lat * DEG_TO_RAD;
        
        north = d_lat * R_EARTH;
        east = d_lon * R_EARTH * std::cos(lat0_rad);
    }
};

} // namespace core
} // namespace drone_follow
