/**
 * CAMERA TO GPS CONVERTER NODE
 * 
 * Subscribes to:
 *  - /camera/detections (vision_msgs/Detection2DArray) - AI detection results
 *  - /camera/camera_info (sensor_msgs/CameraInfo) - Camera calibration
 *  - /gimbal/angles (geometry_msgs/Vector3) - Gimbal pitch/yaw/roll
 *  - /ap/pose/filtered (geometry_msgs/PoseStamped) - Drone pose
 *  - /ap/gps_global_origin/filtered (geographic_msgs/GeoPointStamped) - EKF origin
 * 
 * Publishes to:
 *  - /target/gps (sensor_msgs/NavSatFix) - Converted GPS from camera detection
 *  - /debug/camera_tracking (visualization_msgs/Marker) - Debug visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "drone_offboard/camera_tracking.hpp"
#include <Eigen/Dense>

using namespace camera_tracking;

class CameraToGPSNode : public rclcpp::Node {
public:
    CameraToGPSNode() : Node("camera_to_gps_node") {
        // Parameters
        this->declare_parameter("target_class_id", 0);  // 0 = person (COCO dataset)
        this->declare_parameter("min_confidence", 0.6);
        this->declare_parameter("target_altitude_agl", 0.0);  // Assume target on ground
        this->declare_parameter("publish_rate", 10.0);  // Hz
        
        param_target_class_id_ = this->get_parameter("target_class_id").as_int();
        param_min_confidence_ = this->get_parameter("min_confidence").as_double();
        param_target_altitude_agl_ = this->get_parameter("target_altitude_agl").as_double();
        
        // QoS
        auto qos = rclcpp::SensorDataQoS();
        
        // Subscribers
        sub_detections_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/camera/detections", qos,
            std::bind(&CameraToGPSNode::cb_detections, this, std::placeholders::_1));
        
        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", qos,
            std::bind(&CameraToGPSNode::cb_camera_info, this, std::placeholders::_1));
        
        sub_gimbal_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/gimbal/angles", qos,
            std::bind(&CameraToGPSNode::cb_gimbal, this, std::placeholders::_1));
        
        sub_drone_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ap/pose/filtered", qos,
            std::bind(&CameraToGPSNode::cb_drone_pose, this, std::placeholders::_1));
        
        sub_origin_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
            "/ap/gps_global_origin/filtered", qos,
            std::bind(&CameraToGPSNode::cb_origin, this, std::placeholders::_1));
        
        // Publishers
        pub_target_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/target/gps", 10);
        
        pub_debug_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/debug/camera_tracking", 10);
        
        RCLCPP_INFO(this->get_logger(), "Camera to GPS Converter Node Started");
        RCLCPP_INFO(this->get_logger(), "Target Class: %d, Min Confidence: %.2f",
                    param_target_class_id_, param_min_confidence_);
    }

private:
    void cb_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!camera_calibrated_) {
            CameraIntrinsics intrinsics;
            intrinsics.from_camera_info(*msg);
            converter_.set_camera_intrinsics(intrinsics);
            camera_calibrated_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Camera calibrated: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                        intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
        }
    }
    
    void cb_gimbal(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        gimbal_pitch_deg_ = msg->y;  // Pitch (tilt)
        gimbal_yaw_deg_ = msg->z;    // Yaw (pan)
        gimbal_roll_deg_ = msg->x;   // Roll
        
        converter_.set_gimbal_angles(
            gimbal_pitch_deg_ * M_PI / 180.0,
            gimbal_yaw_deg_ * M_PI / 180.0,
            gimbal_roll_deg_ * M_PI / 180.0
        );
    }
    
    void cb_drone_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Extract position (NED frame from ArduPilot)
        Eigen::Vector3d pos_ned(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );
        
        // Extract yaw from quaternion
        Eigen::Quaterniond q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );
        
        double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
        
        // Assume altitude = position.z (ArduPilot EKF)
        double altitude_agl = pos_ned.z();  // Relative to home
        
        converter_.set_drone_state(pos_ned, yaw_rad, altitude_agl);
        drone_state_valid_ = true;
    }
    
    void cb_origin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {
        if (!origin_set_) {
            gps_converter_.set_origin(
                msg->position.latitude,
                msg->position.longitude,
                msg->position.altitude
            );
            origin_set_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Origin set: lat=%.7f, lon=%.7f, alt=%.2f",
                        msg->position.latitude, msg->position.longitude, msg->position.altitude);
        }
    }
    
    void cb_detections(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        if (!camera_calibrated_ || !drone_state_valid_ || !origin_set_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for camera_info/drone_pose/origin...");
            return;
        }
        
        if (msg->detections.empty()) {
            return;
        }
        
        // Find best detection matching target class
        const vision_msgs::msg::Detection2D* best_detection = nullptr;
        float best_score = 0.0;
        
        for (const auto& detection : msg->detections) {
            if (detection.results.empty()) continue;
            
            // Check class ID and confidence
            int class_id = detection.results[0].id.empty() ? -1 : 
                          std::stoi(detection.results[0].id);
            float score = detection.results[0].score;
            
            if (class_id == param_target_class_id_ && 
                score > param_min_confidence_ && 
                score > best_score) {
                best_detection = &detection;
                best_score = score;
            }
        }
        
        if (!best_detection) {
            return;  // No valid detection
        }
        
        // Extract bounding box (normalized coordinates)
        double bbox_center_x = best_detection->bbox.center.x;
        double bbox_center_y = best_detection->bbox.center.y;
        double bbox_width = best_detection->bbox.size_x;
        double bbox_height = best_detection->bbox.size_y;
        
        // Calculate confidence
        double confidence = converter_.calculate_confidence(
            bbox_center_x, bbox_center_y,
            bbox_width, bbox_height,
            best_score
        );
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Detection: bbox=(%.2f, %.2f), size=(%.3f, %.3f), conf=%.2f",
            bbox_center_x, bbox_center_y, bbox_width, bbox_height, confidence);
        
        // Convert bbox → NED position
        Eigen::Vector3d target_ned;
        bool success = converter_.bbox_to_ned(
            bbox_center_x,
            bbox_center_y,
            true,  // normalized coordinates
            target_ned
        );
        
        if (!success) {
            RCLCPP_WARN(this->get_logger(), "Failed to convert bbox to NED");
            return;
        }
        
        // Convert NED → GPS
        double lat, lon, alt;
        success = gps_converter_.ned_to_gps(target_ned, lat, lon, alt);
        
        if (!success) {
            RCLCPP_WARN(this->get_logger(), "Failed to convert NED to GPS");
            return;
        }
        
        // Publish GPS
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = msg->header.stamp;
        gps_msg.header.frame_id = "wgs84";
        
        gps_msg.latitude = lat;
        gps_msg.longitude = lon;
        gps_msg.altitude = alt;
        
        // Set status based on confidence
        gps_msg.status.status = confidence > 0.7 ? 
            sensor_msgs::msg::NavSatStatus::STATUS_FIX :
            sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        
        // Position covariance (rough estimate based on confidence)
        double pos_std = 2.0 / confidence;  // Lower confidence = higher uncertainty
        gps_msg.position_covariance[0] = pos_std * pos_std;  // Lat
        gps_msg.position_covariance[4] = pos_std * pos_std;  // Lon
        gps_msg.position_covariance[8] = pos_std * pos_std * 2.0;  // Alt (less accurate)
        gps_msg.position_covariance_type = 
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        
        pub_target_gps_->publish(gps_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Published GPS: lat=%.7f, lon=%.7f, alt=%.2f (conf=%.2f)",
            lat, lon, alt, confidence);
        
        // Publish debug marker
        publish_debug_marker(target_ned, confidence);
    }
    
    void publish_debug_marker(const Eigen::Vector3d& target_ned, double confidence) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.ns = "camera_tracking";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = target_ned.x();
        marker.pose.position.y = target_ned.y();
        marker.pose.position.z = -target_ned.z();  // NED to visualization (Z up)
        
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        
        // Color based on confidence (green = high, red = low)
        marker.color.r = 1.0 - confidence;
        marker.color.g = confidence;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        pub_debug_marker_->publish(marker);
    }
    
    // Parameters
    int param_target_class_id_;
    double param_min_confidence_;
    double param_target_altitude_agl_;
    
    // State
    bool camera_calibrated_ = false;
    bool drone_state_valid_ = false;
    bool origin_set_ = false;
    
    double gimbal_pitch_deg_ = 0.0;
    double gimbal_yaw_deg_ = 0.0;
    double gimbal_roll_deg_ = 0.0;
    
    // Converters
    CameraTrackingConverter converter_;
    NEDtoGPSConverter gps_converter_;
    
    // Subscribers
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_detections_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_gimbal_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr sub_origin_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_target_gps_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_debug_marker_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraToGPSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
