# AI Camera-Based Target Tracking for Smart Follow

## 📊 TỔNG QUAN KIẾN TRÚC

### Vấn Đề Cần Giải Quyết:

Khi dùng AI camera để phát hiện vật thể thay vì GPS trực tiếp:

```
┌─────────────────────────────────────────────────────────────┐
│  CAMERA TRACKING vs GPS TRACKING                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  GPS Tracking (Hiện tại):                                  │
│  ┌──────────┐    ┌──────────┐    ┌──────────────┐        │
│  │ GPS App  │───>│ GPS Topic│───>│ smart_follow │        │
│  └──────────┘    └──────────┘    └──────────────┘        │
│     Lat/Lon         Direct          Velocity Calc          │
│                                                             │
│  Camera Tracking (Mới):                                    │
│  ┌──────────┐    ┌───────────┐   ┌────────────┐          │
│  │AI Camera │───>│ Detection │──>│Pixel→GPS   │          │
│  │(YOLOv8)  │    │  (bbox)   │   │ Converter  │          │
│  └──────────┘    └───────────┘   └────────────┘          │
│       ↓               ↓                  ↓                 │
│  Image Frame    2D Coordinates    3D World Position       │
│  (640x480)      (x,y,w,h)         (lat,lon,alt)          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🎯 THUẬT TOÁN TRACKING

### 1. **Hybrid Tracking Architecture**

```
                    ┌──────────────────────────┐
                    │   AI Camera Detection    │
                    │   (YOLOv8/TensorRT)     │
                    └───────────┬──────────────┘
                                │ BBox (x,y,w,h)
                    ┌───────────▼──────────────┐
                    │  Image → World Transform │
                    │  - Camera calibration    │
                    │  - Gimbal angles         │
                    │  - Drone altitude        │
                    └───────────┬──────────────┘
                                │ Target NED position
                    ┌───────────▼──────────────┐
                    │  NED → GPS Conversion    │
                    │  (using EKF origin)      │
                    └───────────┬──────────────┘
                                │ Target GPS
                    ┌───────────▼──────────────┐
                    │  Smart Follow Pipeline   │
                    │  (existing algorithm)    │
                    └──────────────────────────┘
```

### 2. **Camera → GPS Conversion Pipeline**

#### Step 1: Image Coordinates → Normalized Coordinates
```cpp
// Assuming camera center at (cx, cy)
double normalized_x = (bbox_center_x - cx) / fx;  // fx = focal length X
double normalized_y = (bbox_center_y - cy) / fy;  // fy = focal length Y
```

#### Step 2: Apply Gimbal Rotation
```cpp
// Gimbal angles (pitch, yaw từ gimbal feedback)
Eigen::Matrix3d R_gimbal = calculate_gimbal_rotation_matrix(
    gimbal_pitch_rad, 
    gimbal_yaw_rad
);
```

#### Step 3: Ray Casting to Ground
```cpp
// Ray từ camera qua pixel intersect với mặt đất
// Giả định: target trên mặt đất (altitude = terrain_alt)

double drone_alt = drone_relative_alt_;  // từ EKF
double terrain_alt = target_terrain_alt_;  // từ rangefinder/map

// Calculate intersection point
Eigen::Vector3d target_ned = ray_ground_intersection(
    drone_position_ned,
    camera_ray_direction,
    drone_alt - terrain_alt
);
```

#### Step 4: NED → GPS
```cpp
// Sử dụng EKF origin (đã có trong smart_follow_node)
Location target_gps = ned_to_gps(target_ned, origin_lat_, origin_lon_, origin_alt_);
```

---

## 🔧 IMPLEMENTATION

### A. Message Definitions

```cpp
// Tạo custom message cho camera detection
// vision_msgs/msg/Detection2D hoặc custom:

struct CameraDetection {
    rclcpp::Time stamp;
    
    // Bounding box in image coordinates
    int bbox_x, bbox_y, bbox_w, bbox_h;
    
    // Detection confidence
    float confidence;
    
    // Class ID (0=person, 1=car, etc.)
    int class_id;
    
    // Gimbal angles at detection time
    float gimbal_pitch_deg;
    float gimbal_yaw_deg;
    float gimbal_roll_deg;
};
```

### B. Thêm vào smart_follow_node.hpp

```cpp
// Thêm includes
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// Thêm constants
constexpr double CAMERA_TRACKING_TIMEOUT_SEC = 0.5;  // Lost track timeout
constexpr double CAMERA_MIN_CONFIDENCE = 0.6;        // Min detection confidence

// Tracking Mode enum
enum class TrackingMode {
    GPS_ONLY = 0,        // Pure GPS tracking (current)
    CAMERA_ONLY = 1,     // Pure visual tracking
    FUSION = 2           // GPS + Camera fusion (best)
};

// Camera state structure
struct CameraTrackingState {
    bool detection_valid = false;
    rclcpp::Time last_detection;
    
    // Bounding box (normalized 0-1)
    double bbox_center_x = 0.5;
    double bbox_center_y = 0.5;
    double bbox_width = 0.0;
    double bbox_height = 0.0;
    
    // Confidence score
    float confidence = 0.0;
    
    // Estimated target position from camera
    Eigen::Vector3d estimated_pos_ned;
    bool position_valid = false;
};

// Camera parameters
struct CameraIntrinsics {
    double fx = 500.0;  // Focal length X (pixels)
    double fy = 500.0;  // Focal length Y (pixels)
    double cx = 320.0;  // Principal point X
    double cy = 240.0;  // Principal point Y
    int width = 640;
    int height = 480;
};
```

### C. Callbacks và Processing

```cpp
private:
    // === CAMERA TRACKING METHODS ===
    
    // Callback for AI detection results
    void cb_camera_detection(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    
    // Callback for camera calibration info
    void cb_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    // Convert image bbox → NED position
    bool bbox_to_ned_position(
        double bbox_center_x,
        double bbox_center_y,
        double gimbal_pitch,
        double gimbal_yaw,
        Eigen::Vector3d& target_ned_out
    );
    
    // Convert NED → GPS
    void ned_to_gps(
        const Eigen::Vector3d& ned_pos,
        double& lat_out,
        double& lon_out,
        double& alt_out
    );
    
    // Fusion: combine GPS and Camera estimates
    void fusion_gps_camera(
        const Eigen::Vector3d& gps_estimate,
        const Eigen::Vector3d& camera_estimate,
        double gps_confidence,
        double camera_confidence,
        Eigen::Vector3d& fused_estimate
    );
    
    // Members
    CameraTrackingState camera_state_;
    CameraIntrinsics camera_intrinsics_;
    TrackingMode tracking_mode_;
    
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_camera_detection_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
```

---

## 📐 TOÁN HỌC CHI TIẾT

### 1. Camera Projection Model

```
┌─────────────────────────────────────────────┐
│  Pinhole Camera Model                       │
├─────────────────────────────────────────────┤
│                                             │
│  World Point: P = [X, Y, Z] (NED)          │
│  Camera Point: p = [u, v] (pixels)         │
│                                             │
│  Projection:                                │
│  u = fx * (X/Z) + cx                       │
│  v = fy * (Y/Z) + cy                       │
│                                             │
│  Inverse (Ray Casting):                    │
│  X/Z = (u - cx) / fx                       │
│  Y/Z = (v - cy) / fy                       │
│                                             │
│  Ray direction: d = [X/Z, Y/Z, 1]          │
│                                             │
└─────────────────────────────────────────────┘
```

### 2. Ray-Ground Intersection

```cpp
Eigen::Vector3d ray_ground_intersection(
    const Eigen::Vector3d& drone_pos_ned,    // Drone position in NED
    const Eigen::Vector3d& ray_direction,    // Ray from camera
    double relative_height                    // Drone height above ground
) {
    // Normalize ray direction
    Eigen::Vector3d ray_dir_norm = ray_direction.normalized();
    
    // Ground plane: Z = drone_pos_ned.z() - relative_height
    double ground_z = drone_pos_ned.z() - relative_height;
    
    // Calculate t parameter where ray intersects ground
    // drone_pos_ned.z() + t * ray_dir_norm.z() = ground_z
    double t = (ground_z - drone_pos_ned.z()) / ray_dir_norm.z();
    
    // Calculate intersection point
    Eigen::Vector3d target_ned;
    target_ned.x() = drone_pos_ned.x() + t * ray_dir_norm.x();  // North
    target_ned.y() = drone_pos_ned.y() + t * ray_dir_norm.y();  // East
    target_ned.z() = ground_z;                                   // Down
    
    return target_ned;
}
```

### 3. Gimbal Transform

```cpp
Eigen::Matrix3d calculate_gimbal_rotation(
    double pitch_rad,  // Gimbal pitch (tilt down = positive)
    double yaw_rad,    // Gimbal yaw (pan right = positive)
    double roll_rad = 0.0
) {
    // Rotation matrices (ZYX Euler angles)
    Eigen::Matrix3d R_yaw;
    R_yaw << cos(yaw_rad), -sin(yaw_rad), 0,
             sin(yaw_rad),  cos(yaw_rad), 0,
             0,             0,             1;
    
    Eigen::Matrix3d R_pitch;
    R_pitch << cos(pitch_rad), 0, sin(pitch_rad),
               0,              1, 0,
              -sin(pitch_rad), 0, cos(pitch_rad);
    
    Eigen::Matrix3d R_roll;
    R_roll << 1, 0,            0,
              0, cos(roll_rad), -sin(roll_rad),
              0, sin(roll_rad),  cos(roll_rad);
    
    // Combined rotation: R = R_yaw * R_pitch * R_roll
    return R_yaw * R_pitch * R_roll;
}
```

---

## 🎮 FUSION STRATEGY

### Strategy 1: Confidence-Based Weighted Fusion

```cpp
void fusion_gps_camera(
    const Eigen::Vector3d& gps_estimate,
    const Eigen::Vector3d& camera_estimate,
    double gps_confidence,      // 0.0 - 1.0
    double camera_confidence,   // 0.0 - 1.0
    Eigen::Vector3d& fused_estimate
) {
    // Normalize weights
    double total_conf = gps_confidence + camera_confidence;
    double w_gps = gps_confidence / total_conf;
    double w_cam = camera_confidence / total_conf;
    
    // Weighted fusion
    fused_estimate = w_gps * gps_estimate + w_cam * camera_estimate;
}
```

### Strategy 2: Kalman Filter Fusion (Advanced)

```cpp
// State: [x, y, z, vx, vy, vz]
// GPS measurement: [x, y, z]
// Camera measurement: [x, y] (only horizontal)

class TargetKalmanFilter {
public:
    void predict(double dt);
    void update_gps(const Eigen::Vector3d& gps_meas);
    void update_camera(const Eigen::Vector2d& camera_meas);
    Eigen::Vector3d get_state() const;
};
```

---

## 📊 TRACKING MODES

### Mode 1: GPS Only (Hiện tại)
```yaml
tracking_mode: 0  # GPS_ONLY
# Dùng khi: outdoor, GPS tốt, target xa
```

### Mode 2: Camera Only
```yaml
tracking_mode: 1  # CAMERA_ONLY
# Dùng khi: indoor, GPS denied, target gần (<50m)
# Yêu cầu: gimbal stable, camera calibration tốt
```

### Mode 3: Fusion (Recommended)
```yaml
tracking_mode: 2  # FUSION
# Kết hợp GPS (long-range) + Camera (precision)
# GPS confidence cao khi target xa
# Camera confidence cao khi target gần và trong FOV
```

### Confidence Calculation:

```cpp
double calculate_gps_confidence() {
    double age = (now - target_.last_update).seconds();
    double conf = 1.0 - std::min(age / param_timeout_, 1.0);
    
    // Penalize high velocity (GPS lag)
    double vel = target_map_.vel_enu.head<2>().norm();
    conf *= std::max(0.0, 1.0 - vel / MAX_TARGET_VELOCITY);
    
    return conf;
}

double calculate_camera_confidence() {
    // Detection confidence from AI
    double conf = camera_state_.confidence;
    
    // Penalize if bbox near image edge
    double edge_penalty = calculate_edge_distance_penalty(
        camera_state_.bbox_center_x,
        camera_state_.bbox_center_y
    );
    conf *= edge_penalty;
    
    // Penalize if bbox too small (far away)
    double size = camera_state_.bbox_width * camera_state_.bbox_height;
    double size_penalty = std::min(size / 0.05, 1.0);  // 5% of image
    conf *= size_penalty;
    
    return conf;
}
```

---

## 🚀 INTEGRATION WORKFLOW

### Phase 1: Camera-Only Tracking (Simplified)

```cpp
// 1. Subscribe to AI detection
sub_camera_detection_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/camera/detections", 10,
    std::bind(&SmartFollowNode::cb_camera_detection, this, _1)
);

// 2. Convert detection → GPS
void cb_camera_detection(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (msg->detections.empty()) {
        camera_state_.detection_valid = false;
        return;
    }
    
    // Get best detection (highest confidence)
    auto best = std::max_element(msg->detections.begin(), msg->detections.end(),
        [](auto& a, auto& b) { return a.results[0].score < b.results[0].score; });
    
    // Convert bbox → NED position
    Eigen::Vector3d target_ned;
    bool valid = bbox_to_ned_position(
        best->bbox.center.x,
        best->bbox.center.y,
        gimbal_pitch_rad_,
        gimbal_yaw_rad_,
        target_ned
    );
    
    if (valid) {
        // Convert NED → GPS
        double lat, lon, alt;
        ned_to_gps(target_ned, lat, lon, alt);
        
        // Update target state (như GPS callback)
        update_target_from_camera(lat, lon, alt);
    }
}
```

### Phase 2: GPS + Camera Fusion

```cpp
NavCommand calculate_kinematics() {
    Eigen::Vector3d target_pos;
    
    // Get GPS estimate
    Eigen::Vector3d gps_estimate = target_map_.pos_enu;
    double gps_conf = calculate_gps_confidence();
    
    // Get Camera estimate
    Eigen::Vector3d camera_estimate = camera_state_.estimated_pos_ned;
    double camera_conf = calculate_camera_confidence();
    
    // Fusion
    if (tracking_mode_ == TrackingMode::FUSION) {
        fusion_gps_camera(gps_estimate, camera_estimate, 
                         gps_conf, camera_conf, target_pos);
    } else if (tracking_mode_ == TrackingMode::CAMERA_ONLY) {
        target_pos = camera_estimate;
    } else {
        target_pos = gps_estimate;
    }
    
    // Continue with existing algorithm...
}
```

---

## ⚙️ CONFIGURATION

```yaml
smart_follow_node:
  ros__parameters:
    # Tracking mode
    tracking_mode: 2  # 0=GPS, 1=Camera, 2=Fusion
    
    # Camera parameters
    camera_tracking_enable: true
    camera_timeout: 0.5               # Lost track timeout (s)
    camera_min_confidence: 0.6        # Min AI confidence
    camera_fusion_weight: 0.5         # Camera weight in fusion (0-1)
    
    # Camera calibration (auto from /camera/camera_info)
    camera_fx: 500.0
    camera_fy: 500.0
    camera_cx: 320.0
    camera_cy: 240.0
    
    # Gimbal compensation
    use_gimbal_feedback: true         # Use gimbal angles from /gimbal/angles
```

---

## 🧪 TESTING

### Test 1: Camera Calibration Check
```bash
# 1. Publish camera info
ros2 topic pub /camera/camera_info sensor_msgs/msg/CameraInfo "{...}"

# 2. Point camera at known GPS location
# 3. Verify calculated GPS matches ground truth
```

### Test 2: Tracking Accuracy
```bash
# Compare GPS vs Camera estimates
ros2 topic echo /debug/gps_estimate
ros2 topic echo /debug/camera_estimate
ros2 topic echo /debug/fused_estimate
```

---

## 🎯 ADVANTAGES vs GPS-ONLY

| Feature | GPS Only | Camera + GPS Fusion |
|---------|----------|---------------------|
| **Precision** | ±2-5m | ±0.5-2m (close range) |
| **Update rate** | 1-5Hz | 10-30Hz |
| **Indoor** | ❌ No | ✅ Yes (camera only) |
| **Occlusion handling** | ✅ Good | ⚠️ Need fallback |
| **Range** | Unlimited | <100m (camera FOV) |
| **Latency** | High (GPS lag) | Low (visual direct) |

---

## 📚 REFERENCES

- **YOLOv8:** https://github.com/ultralytics/ultralytics
- **vision_msgs:** https://github.com/ros-perception/vision_msgs
- **Camera Calibration:** http://wiki.ros.org/camera_calibration

