/**
 * CAMERA TRACKING MODULE FOR SMART FOLLOW
 * 
 * Converts AI camera detections (bounding boxes) to GPS coordinates
 * for integration with smart_follow_node
 */

#ifndef CAMERA_TRACKING_HPP_
#define CAMERA_TRACKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace camera_tracking {

// ============================================================================
// Camera Intrinsics
// ============================================================================
struct CameraIntrinsics {
    double fx = 500.0;  // Focal length X (pixels)
    double fy = 500.0;  // Focal length Y (pixels)
    double cx = 320.0;  // Principal point X (image center)
    double cy = 240.0;  // Principal point Y (image center)
    int width = 640;    // Image width
    int height = 480;   // Image height
    
    // Distortion coefficients (if needed)
    std::vector<double> distortion_coeffs;
    
    // Load from ROS CameraInfo message
    void from_camera_info(const sensor_msgs::msg::CameraInfo& msg) {
        fx = msg.k[0];  // K[0,0]
        fy = msg.k[4];  // K[1,1]
        cx = msg.k[2];  // K[0,2]
        cy = msg.k[5];  // K[1,2]
        width = msg.width;
        height = msg.height;
        distortion_coeffs = msg.d;
    }
};

// ============================================================================
// Detection State
// ============================================================================
struct DetectionState {
    bool valid = false;
    rclcpp::Time timestamp;
    
    // Bounding box (normalized 0-1)
    double bbox_center_x = 0.5;
    double bbox_center_y = 0.5;
    double bbox_width = 0.0;
    double bbox_height = 0.0;
    
    // Bounding box (pixel coordinates)
    double bbox_pixel_x = 0.0;
    double bbox_pixel_y = 0.0;
    double bbox_pixel_w = 0.0;
    double bbox_pixel_h = 0.0;
    
    // Detection metadata
    float confidence = 0.0;
    int class_id = -1;
    std::string class_name = "";
    
    // Estimated 3D position
    Eigen::Vector3d position_ned;  // North-East-Down
    bool position_valid = false;
    
    // Confidence factors
    double edge_distance_penalty = 1.0;  // 0-1, penalize edge detections
    double size_penalty = 1.0;           // 0-1, penalize small detections
    double overall_confidence = 0.0;     // Combined confidence
};

// ============================================================================
// Camera Tracking Converter
// ============================================================================
class CameraTrackingConverter {
public:
    CameraTrackingConverter() = default;
    
    // Set camera intrinsics
    void set_camera_intrinsics(const CameraIntrinsics& intrinsics) {
        intrinsics_ = intrinsics;
    }
    
    // Set current gimbal angles (from gimbal feedback)
    void set_gimbal_angles(double pitch_rad, double yaw_rad, double roll_rad = 0.0) {
        gimbal_pitch_rad_ = pitch_rad;
        gimbal_yaw_rad_ = yaw_rad;
        gimbal_roll_rad_ = roll_rad;
    }
    
    // Set current drone state
    void set_drone_state(
        const Eigen::Vector3d& position_ned,
        double yaw_rad,
        double altitude_agl  // Above Ground Level
    ) {
        drone_position_ned_ = position_ned;
        drone_yaw_rad_ = yaw_rad;
        drone_altitude_agl_ = altitude_agl;
    }
    
    /**
     * Convert bounding box to NED position
     * 
     * @param bbox_center_x Bbox center X (normalized 0-1 or pixels)
     * @param bbox_center_y Bbox center Y (normalized 0-1 or pixels)
     * @param is_normalized True if coords are normalized, false if pixels
     * @param target_ned_out Output NED position
     * @return true if conversion successful
     */
    bool bbox_to_ned(
        double bbox_center_x,
        double bbox_center_y,
        bool is_normalized,
        Eigen::Vector3d& target_ned_out
    ) {
        // Convert to pixel coordinates if normalized
        double pixel_x, pixel_y;
        if (is_normalized) {
            pixel_x = bbox_center_x * intrinsics_.width;
            pixel_y = bbox_center_y * intrinsics_.height;
        } else {
            pixel_x = bbox_center_x;
            pixel_y = bbox_center_y;
        }
        
        // Step 1: Pixel → Normalized camera coordinates
        double norm_x = (pixel_x - intrinsics_.cx) / intrinsics_.fx;
        double norm_y = (pixel_y - intrinsics_.cy) / intrinsics_.fy;
        
        // Step 2: Create ray direction in camera frame
        // Camera frame: X=right, Y=down, Z=forward
        Eigen::Vector3d ray_camera(norm_x, norm_y, 1.0);
        ray_camera.normalize();
        
        // Step 3: Apply gimbal rotation
        // Transform ray from camera frame to body frame
        Eigen::Matrix3d R_gimbal = calculate_gimbal_rotation();
        Eigen::Vector3d ray_body = R_gimbal * ray_camera;
        
        // Step 4: Apply drone rotation
        // Transform ray from body frame to NED frame
        Eigen::Matrix3d R_drone = calculate_drone_rotation();
        Eigen::Vector3d ray_ned = R_drone * ray_body;
        
        // Step 5: Ray-ground intersection
        // Assume target is on the ground (altitude_agl meters below drone)
        target_ned_out = ray_ground_intersection(
            drone_position_ned_,
            ray_ned,
            drone_altitude_agl_
        );
        
        return true;
    }
    
    /**
     * Calculate confidence score for detection
     * 
     * @param bbox_x Bbox center X (normalized)
     * @param bbox_y Bbox center Y (normalized)
     * @param bbox_w Bbox width (normalized)
     * @param bbox_h Bbox height (normalized)
     * @param detection_confidence AI detection confidence (0-1)
     * @return Overall confidence (0-1)
     */
    double calculate_confidence(
        double bbox_x,
        double bbox_y,
        double bbox_w,
        double bbox_h,
        double detection_confidence
    ) {
        // Factor 1: Detection confidence from AI
        double conf = detection_confidence;
        
        // Factor 2: Edge distance penalty
        // Penalize detections near image edges (may be cut off)
        double edge_margin = 0.1;  // 10% margin
        double dx_left = bbox_x - bbox_w / 2.0;
        double dx_right = 1.0 - (bbox_x + bbox_w / 2.0);
        double dy_top = bbox_y - bbox_h / 2.0;
        double dy_bottom = 1.0 - (bbox_y + bbox_h / 2.0);
        
        double edge_dist = std::min({dx_left, dx_right, dy_top, dy_bottom});
        double edge_penalty = std::clamp(edge_dist / edge_margin, 0.0, 1.0);
        conf *= edge_penalty;
        
        // Factor 3: Size penalty
        // Penalize very small detections (far away, unreliable)
        double bbox_area = bbox_w * bbox_h;
        double min_area = 0.001;  // 0.1% of image
        double size_penalty = std::clamp(bbox_area / min_area, 0.0, 1.0);
        conf *= size_penalty;
        
        // Factor 4: Aspect ratio check (optional)
        // Reject detections with unrealistic aspect ratios
        double aspect_ratio = bbox_w / (bbox_h + 1e-6);
        if (aspect_ratio < 0.2 || aspect_ratio > 5.0) {
            conf *= 0.5;  // Penalize strange aspect ratios
        }
        
        return conf;
    }

private:
    // Camera parameters
    CameraIntrinsics intrinsics_;
    
    // Gimbal state
    double gimbal_pitch_rad_ = 0.0;
    double gimbal_yaw_rad_ = 0.0;
    double gimbal_roll_rad_ = 0.0;
    
    // Drone state
    Eigen::Vector3d drone_position_ned_;
    double drone_yaw_rad_ = 0.0;
    double drone_altitude_agl_ = 10.0;  // Default 10m
    
    /**
     * Calculate gimbal rotation matrix
     * Transforms from camera frame to body frame
     */
    Eigen::Matrix3d calculate_gimbal_rotation() {
        // Gimbal convention: Pitch (tilt), Yaw (pan), Roll
        // Camera points forward when gimbal at zero
        
        double p = gimbal_pitch_rad_;
        double y = gimbal_yaw_rad_;
        double r = gimbal_roll_rad_;
        
        // Rotation matrices
        Eigen::Matrix3d R_pitch;
        R_pitch << cos(p), 0, sin(p),
                   0,      1, 0,
                  -sin(p), 0, cos(p);
        
        Eigen::Matrix3d R_yaw;
        R_yaw << cos(y), -sin(y), 0,
                 sin(y),  cos(y), 0,
                 0,       0,      1;
        
        Eigen::Matrix3d R_roll;
        R_roll << 1, 0,       0,
                  0, cos(r), -sin(r),
                  0, sin(r),  cos(r);
        
        // Combined: R = R_yaw * R_pitch * R_roll
        return R_yaw * R_pitch * R_roll;
    }
    
    /**
     * Calculate drone rotation matrix
     * Transforms from body frame to NED frame
     */
    Eigen::Matrix3d calculate_drone_rotation() {
        // Drone yaw (heading)
        double yaw = drone_yaw_rad_;
        
        Eigen::Matrix3d R_yaw;
        R_yaw << cos(yaw), -sin(yaw), 0,
                 sin(yaw),  cos(yaw), 0,
                 0,         0,        1;
        
        return R_yaw;
    }
    
    /**
     * Ray-ground intersection
     * Find where camera ray intersects the ground plane
     */
    Eigen::Vector3d ray_ground_intersection(
        const Eigen::Vector3d& drone_pos_ned,
        const Eigen::Vector3d& ray_direction,
        double altitude_agl
    ) {
        // Ground plane is at Z = drone_pos_ned.z() - altitude_agl
        // (NED frame: Z points down, so ground is at higher Z value)
        
        double ground_z = drone_pos_ned.z() + altitude_agl;  // Down = positive
        
        // Ray equation: P = P0 + t * d
        // Find t where P.z = ground_z
        // drone_pos_ned.z() + t * ray_direction.z() = ground_z
        
        double t = (ground_z - drone_pos_ned.z()) / ray_direction.z();
        
        // Check if intersection is valid (t > 0, ray points downward)
        if (t < 0 || ray_direction.z() < 0) {
            // Invalid: ray doesn't intersect ground (points upward)
            return drone_pos_ned;  // Fallback: return drone position
        }
        
        // Calculate intersection point
        Eigen::Vector3d target_ned;
        target_ned.x() = drone_pos_ned.x() + t * ray_direction.x();
        target_ned.y() = drone_pos_ned.y() + t * ray_direction.y();
        target_ned.z() = ground_z;
        
        return target_ned;
    }
};

// ============================================================================
// GPS Converter
// ============================================================================
class NEDtoGPSConverter {
public:
    NEDtoGPSConverter() = default;
    
    // Set EKF origin (from ArduPilot)
    void set_origin(double lat, double lon, double alt) {
        origin_lat_ = lat;
        origin_lon_ = lon;
        origin_alt_ = alt;
        origin_set_ = true;
    }
    
    /**
     * Convert NED position to GPS coordinates
     * 
     * @param ned_pos Position in NED frame (meters)
     * @param lat_out Output latitude (degrees)
     * @param lon_out Output longitude (degrees)
     * @param alt_out Output altitude MSL (meters)
     * @return true if conversion successful
     */
    bool ned_to_gps(
        const Eigen::Vector3d& ned_pos,
        double& lat_out,
        double& lon_out,
        double& alt_out
    ) {
        if (!origin_set_) {
            return false;
        }
        
        const double R_EARTH = 6378137.0;  // Earth radius (WGS84)
        const double DEG_TO_RAD = M_PI / 180.0;
        
        // NED to LLA conversion
        double d_north = ned_pos.x();  // meters
        double d_east = ned_pos.y();   // meters
        double d_down = ned_pos.z();   // meters (positive = down)
        
        // Latitude change
        double d_lat_rad = d_north / R_EARTH;
        lat_out = origin_lat_ + d_lat_rad / DEG_TO_RAD;
        
        // Longitude change (account for latitude)
        double lat_rad = origin_lat_ * DEG_TO_RAD;
        double d_lon_rad = d_east / (R_EARTH * cos(lat_rad));
        lon_out = origin_lon_ + d_lon_rad / DEG_TO_RAD;
        
        // Altitude (MSL)
        alt_out = origin_alt_ - d_down;  // Down is positive in NED
        
        return true;
    }

private:
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_alt_ = 0.0;
    bool origin_set_ = false;
};

}  // namespace camera_tracking

#endif  // CAMERA_TRACKING_HPP_
