# Quick Start: AI Camera Tracking with Smart Follow

## 📋 YOLO Detection → GPS → Smart Follow Pipeline

```
┌────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  YOLOv8 Node   │───>│ Camera2GPS Node  │───>│ Smart Follow Node│
│  (AI Camera)   │    │  (This Package)  │    │  (Control)       │
└────────────────┘    └──────────────────┘    └──────────────────┘
   Detections          Convert to GPS           Follow Target
```

---

## 🚀 SETUP

### 1. Install Dependencies

```bash
# Vision messages (for detection data)
sudo apt install ros-humble-vision-msgs

# YOLOv8 ROS 2 (example AI detector)
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/yolov8_ros.git
cd ~/ros2_ws
colcon build --packages-select yolov8_ros

# OpenCV (if needed)
sudo apt install ros-humble-cv-bridge
```

### 2. Camera Calibration

```bash
# Calibrate camera using ROS camera_calibration
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --ros-args -r image:=/camera/image_raw

# Save calibration to YAML
# File sẽ ở: ~/.ros/camera_info/camera_name.yaml
```

### 3. Build Camera2GPS Node

```bash
cd /home/xb/ardu_ws
colcon build --packages-select drone_offboard

source install/setup.bash
```

---

## 🎮 RUNNING

### Terminal 1: Camera + YOLOv8

```bash
# Start camera driver
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p framerate:=30.0 \
  -p image_width:=640 \
  -p image_height:=480

# OR use RealSense
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: YOLOv8 detector
ros2 launch yolov8_ros yolov8.launch.py \
  model:=yolov8n.pt \
  threshold:=0.6 \
  input_image_topic:=/camera/image_raw \
  enable_segmentation:=false
```

### Terminal 2: Camera2GPS Converter

```bash
ros2 run drone_offboard camera_to_gps_node --ros-args \
  -p target_class_id:=0 \        # 0 = person (COCO)
  -p min_confidence:=0.6 \
  -p target_altitude_agl:=0.0    # Target on ground
```

### Terminal 3: Smart Follow Node

```bash
ros2 run drone_offboard smart_follow_node --ros-args \
  --params-file config/smart_follow_params.yaml
```

### Terminal 4: ArduPilot DDS Bridge

```bash
# On flight controller or companion computer
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
```

---

## 🔍 VERIFICATION

### Check Data Flow:

```bash
# 1. Verify camera detections
ros2 topic echo /camera/detections --no-arr

# 2. Verify GPS conversion
ros2 topic echo /target/gps

# 3. Verify follow commands
ros2 topic echo /ap/cmd_vel

# 4. Visualize in RViz
ros2 run rviz2 rviz2
# Add Marker topic: /debug/camera_tracking
```

### Expected Output:

```bash
# /camera/detections
detections:
  - bbox:
      center: {x: 0.5, y: 0.6}
      size_x: 0.15
      size_y: 0.25
    results:
      - id: "0"  # person
        score: 0.85

# /target/gps
latitude: 10.8231234
longitude: 106.6289567
altitude: 15.3
status:
  status: 0  # STATUS_FIX

# /ap/cmd_vel
twist:
  linear:
    x: 1.2   # Follow forward
    y: -0.3  # Adjust left
    z: 0.1   # Climb slightly
  angular:
    z: 0.05  # Yaw adjustment
```

---

## ⚙️ TUNING

### 1. Camera Calibration Accuracy

```yaml
# If GPS positions are consistently offset:
# - Recalibrate camera with more images
# - Check gimbal angle feedback accuracy
# - Verify drone altitude source
```

### 2. Detection Confidence

```yaml
# config/camera_tracking.yaml
camera_to_gps_node:
  ros__parameters:
    min_confidence: 0.6     # Lower = more detections but noisier
    target_class_id: 0      # COCO: 0=person, 2=car, 16=dog
```

### 3. Altitude Estimation

```bash
# If target appears at wrong GPS location:
# - Check drone altitude: should be AGL (above ground)
# - Use rangefinder for accurate AGL
# - Or use terrain database

ros2 param set /camera_to_gps_node target_altitude_agl 0.0  # Ground
ros2 param set /camera_to_gps_node target_altitude_agl 1.5  # Person height
```

### 4. Gimbal Compensation

```yaml
# Verify gimbal feedback:
ros2 topic echo /gimbal/angles

# Output should match physical gimbal:
# x: 0.0    # Roll
# y: -30.0  # Pitch (negative = down)
# z: 15.0   # Yaw (positive = right)
```

---

## 🧪 TESTING SCENARIOS

### Test 1: Static Person on Ground

```bash
# Setup:
# - Person standing still
# - Drone hovering at 10m altitude
# - Gimbal pointing down ~60°

# Expected:
# - GPS should be within 1-2m of person's actual GPS
# - Velocity should be ~0 m/s
```

### Test 2: Walking Person

```bash
# Setup:
# - Person walking at 1 m/s
# - Drone following at 5m distance
# - Camera tracking enabled

# Expected:
# - GPS updates at camera rate (10-30Hz)
# - Velocity estimate matches walking speed
# - Drone follows smoothly
```

### Test 3: GPS Denied (Indoor)

```bash
# Disable GPS tracking:
ros2 param set /smart_follow_node tracking_mode 1  # CAMERA_ONLY

# Expected:
# - Follow works without GPS
# - Relies purely on camera detection
# - May drift if target leaves FOV
```

---

## 🐛 TROUBLESHOOTING

### Issue 1: No GPS Published

**Symptoms:**
```bash
ros2 topic echo /target/gps
# No messages
```

**Checklist:**
- [ ] Camera publishing images: `ros2 topic hz /camera/image_raw`
- [ ] YOLOv8 detecting: `ros2 topic hz /camera/detections`
- [ ] Camera calibration loaded: check `/camera/camera_info`
- [ ] Drone pose available: `ros2 topic hz /ap/pose/filtered`
- [ ] Origin set: `ros2 topic hz /ap/gps_global_origin/filtered`

### Issue 2: GPS Position Incorrect

**Symptoms:**
```bash
# Target GPS is offset by 10-50m
```

**Fixes:**
1. **Recalibrate camera:**
   ```bash
   ros2 run camera_calibration cameracalibrator ...
   ```

2. **Check gimbal angles:**
   ```bash
   # Manual test: point gimbal at known landmark
   # Verify calculated GPS matches landmark GPS
   ```

3. **Verify altitude source:**
   ```bash
   # ArduPilot altitude should be AGL (above ground)
   # NOT MSL (mean sea level)
   ros2 topic echo /ap/pose/filtered
   # pose.position.z should be ~10m when hovering at 10m AGL
   ```

### Issue 3: Jittery GPS

**Symptoms:**
```bash
# GPS jumps around rapidly
```

**Fixes:**
1. **Enable jitter correction:**
   ```yaml
   jitter_correction_enable: true
   jitter_max_lag_ms: 200  # Camera has low latency
   ```

2. **Increase alpha-beta filter:**
   ```yaml
   filter_alpha: 0.7  # More filtering
   filter_beta: 0.05  # Slower velocity updates
   ```

3. **Lower camera rate:**
   ```bash
   # Reduce to 10Hz for more stable estimates
   ros2 param set /camera_node framerate 10.0
   ```

---

## 📊 PERFORMANCE METRICS

| Metric | GPS Only | Camera + GPS Fusion |
|--------|----------|---------------------|
| **Update Rate** | 1-5 Hz | 10-30 Hz |
| **Precision (outdoor)** | ±2-5m | ±1-3m |
| **Precision (close)** | ±2-5m | ±0.5-1m |
| **Latency** | 200-500ms | 50-100ms |
| **Indoor capability** | ❌ No | ✅ Yes |
| **CPU usage** | Low | Medium |

---

## 📚 COCO Dataset Class IDs

Common targets for tracking:

```python
COCO_CLASSES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
    16: "dog",
    17: "horse",
    18: "sheep",
    19: "cow"
}
```

---

## 🎯 ADVANCED: Custom Detection Model

### Train YOLOv8 on Custom Dataset:

```bash
# 1. Prepare dataset (images + labels)
# 2. Train YOLOv8
python train.py --data custom.yaml --epochs 100

# 3. Export to ONNX/TensorRT
yolo export model=best.pt format=onnx

# 4. Use in ROS
ros2 launch yolov8_ros yolov8.launch.py model:=best.onnx
```

---

## 🔗 INTEGRATION WITH SMART_FOLLOW_NODE

### Option A: Direct Replacement (Simple)

Camera2GPS node publishes to `/target/gps` → Smart Follow subscribes

✅ **Pros:** No code changes, drop-in replacement  
⚠️ **Cons:** No fusion, pure camera or pure GPS

### Option B: Fusion Mode (Advanced)

Modify `smart_follow_node.cpp` to subscribe to both:
- `/target/gps` (from phone/GPS module)
- `/target/camera_gps` (from camera detection)

Implement fusion logic:
```cpp
if (tracking_mode_ == FUSION) {
    fused_pos = weighted_fusion(gps_pos, camera_pos);
}
```

✅ **Pros:** Best of both worlds  
⚠️ **Cons:** Requires code modification

---

## 📖 NEXT STEPS

1. **Camera Calibration** - Critical for accuracy
2. **Gimbal Integration** - Accurate angle feedback
3. **Altitude Source** - Use rangefinder for AGL
4. **Fusion Tuning** - Weight GPS vs Camera based on range
5. **Fail-Safe** - Fallback to GPS if camera lost

Happy Tracking! 🚁📹
