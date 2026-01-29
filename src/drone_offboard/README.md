# Smart Follow Node for ArduPilot (ROS 2 Native DDS)

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-DDS-orange)](https://ardupilot.org/dev/docs/ros2.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

A professional-grade target following node for ArduPilot drones using ROS 2 Native DDS (no MAVROS required). Implements advanced kinematics from ArduPilot's AP_Follow library with additional enhancements.

## 🎯 Features

### Core Features (Ported from AP_Follow)
- **Kinematic Prediction**: Position prediction using `P = P₀ + V·t + ½·A·t²`
- **Jerk-Limited Input Shaping**: Separate limits for NE (horizontal), D (vertical), and H (heading)
- **Configurable Timeout**: AP_FOLLOW style timeout with error detection
- **Position P Controller**: Proportional control with feedforward
- **Map-Frame Architecture**: Fixed EKF origin as ENU reference (no GPS drift!)

### Enhanced Features (Beyond AP_Follow)
| Feature | Description |
|---------|-------------|
| **Alpha-Beta Filter** | Smart velocity estimation from GPS (replaces simple low-pass) |
| **Lateral Feed-Forward** | Prevents "swing out" when target turns |
| **Centripetal Compensation** | Compensates centripetal acceleration during circular motion |
| **Terrain Following** | Maintains relative altitude above target (not fixed altitude) |
| **Adaptive Distance** | Follow distance varies with target speed |
| **Heading Blend** | Hybrid gimbal tracking + heading following |
| **50Hz Control Loop** | Smoother than typical 20Hz implementations |
| **MapPosition Struct** | Drone & Target in same ENU frame for clean error calculation |

## 📐 Architecture

### Coordinate Frame Design (Critical!)

```
                 /ap/gps_global_origin/filtered
                            ↓
                    (lat0, lon0, alt0)
                   ═══════════════════
                   FIXED EKF ORIGIN
                   (Set once at boot)
                            ↓
        ┌─────────────── ENU Map Frame ───────────────┐
        │                                             │
Drone:  │  GPS → gps_to_enu(origin) → (x_d, y_d)     │
Target: │  GPS → gps_to_enu(origin) → (x_t, y_t)     │
        │                                             │
        │         Error = target_enu - drone_enu      │
        └─────────────────────────────────────────────┘
                            ↓
                    Body-frame control
```

> ⚠️ **Why fixed origin matters**: Using drone GPS as reference causes "coordinate drift" because GPS noise makes the reference point move. The EKF origin from ArduPilot is fixed and consistent with `/ap/pose/filtered`.

### Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SmartFollowNode (ROS2 DDS)                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌────────────────────┐                                             │
│  │/ap/gps_global_origin│  ← FIXED REFERENCE (received once)         │
│  │  (GeoPointStamped)  │                                            │
│  └─────────┬──────────┘                                             │
│            ↓                                                        │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐   │
│  │ /ap/navsat   │    │ /target/gps  │    │ /gimbal/angle_error  │   │
│  │ (NavSatFix)  │    │ (NavSatFix)  │    │ (Point)              │   │
│  └──────┬───────┘    └──────┬───────┘    └──────────┬───────────┘   │
│         │                   │                       │               │
│         ▼                   ▼                       ▼               │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │         GPS → ENU Conversion (using fixed origin)             │   │
│  │         + Alpha-Beta Filters (Velocity Estimation)            │   │
│  └──────────────────────────┬───────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                  Kinematic Prediction                         │   │
│  │            P_future = P + V·t + 0.5·A·t²                      │   │
│  └──────────────────────────┬───────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              P + FeedForward Controller                       │   │
│  │     cmd = Kp·error + FF_forward + FF_lateral + FF_centripetal │   │
│  └──────────────────────────┬───────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │           2-Stage Kinematic Input Shaping                     │   │
│  │         Stage 1: Jerk Limiting → Stage 2: Accel Limiting      │   │
│  └──────────────────────────┬───────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│                    ┌────────────────┐                               │
│                    │  /ap/cmd_vel   │                               │
│                    │ (TwistStamped) │                               │
│                    └────────────────┘                               │
└─────────────────────────────────────────────────────────────────────┘
```

## 📡 ROS 2 Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `/ap/gps_global_origin/filtered` | `geographic_msgs/GeoPointStamped` | **EKF Origin** - Fixed WGS84 reference point (Critical!) |
| `/ap/navsat/navsat_fix` | `sensor_msgs/NavSatFix` | Drone GPS position (ArduPilot DDS) |
| `/ap/pose/filtered` | `geometry_msgs/PoseStamped` | Drone pose with orientation (ArduPilot DDS) |
| `/target/gps` | `sensor_msgs/NavSatFix` | Target GPS position (from phone app or GPS module) |
| `/gimbal/angle_error` | `geometry_msgs/Point` | Gimbal pan/tilt error in degrees (from vision node) |
| `/ap/status/flight_mode` | `std_msgs/String` | Flight mode (must be "GUIDED") |
| `/ap/status/armed` | `std_msgs/Bool` | Armed status |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `/ap/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands in body frame |

## ⚙️ Parameters

### Basic Follow Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `follow_dist` | 5.0 | m | Distance to maintain behind target |
| `follow_height` | 3.0 | m | Height above target (terrain follow) |
| `prediction_time` | 0.5 | s | Look-ahead time for prediction |
| `timeout` | 3.0 | s | Target timeout (AP_FOLLOW style) |

### Controller Gains

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp_pos` | 0.1 | Position P gain (AP_FOLLOW_POS_P_DEFAULT) |
| `kp_vel_z` | 1.5 | Vertical velocity P gain |
| `kp_yaw` | 0.05 | Yaw rate P gain (gimbal tracking) |
| `ff_lateral_gain` | 0.85 | Lateral feed-forward gain (0.0 - 1.0) |
| `ff_centripetal_gain` | 2.0 | Centripetal compensation gain |

### Kinematic Constraints (AP_Follow Style)

| Parameter | Default | Unit | AP_Follow Equivalent |
|-----------|---------|------|---------------------|
| `accel_max_ne` | 2.5 | m/s² | FOLL_ACCEL_NE |
| `jerk_max_ne` | 5.0 | m/s³ | FOLL_JERK_NE |
| `accel_max_d` | 2.5 | m/s² | FOLL_ACCEL_D |
| `jerk_max_d` | 5.0 | m/s³ | FOLL_JERK_D |
| `accel_max_h` | 90.0 | deg/s² | FOLL_ACCEL_H |
| `jerk_max_h` | 360.0 | deg/s³ | FOLL_JERK_H |
| `gimbal_deadzone` | 5.0 | deg | Gimbal error deadzone |

### Alpha-Beta Filter

| Parameter | Default | Description |
|-----------|---------|-------------|
| `filter_alpha` | 0.6 | Measurement trust (0.0 - 1.0) |
| `filter_beta` | 0.1 | Velocity update rate |

### Advanced Features

| Parameter | Default | Description |
|-----------|---------|-------------|
| `adaptive_distance_enable` | false | Enable speed-based follow distance |
| `follow_dist_min` | 3.0 | Minimum follow distance (m) |
| `follow_dist_max` | 15.0 | Maximum follow distance (m) |
| `distance_speed_gain` | 1.5 | Distance = min + gain × speed |
| `heading_blend_enable` | false | Enable heading blending |
| `heading_blend_weight` | 0.3 | Gimbal vs heading blend (0=gimbal, 1=heading) |
| `heading_follow_kp` | 0.5 | Heading error P gain |
| `terrain_follow_enable` | true | Follow target altitude vs fixed altitude |

## 🔧 Build & Run

### Prerequisites

```bash
# Install dependencies
sudo apt install libeigen3-dev
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select drone_offboard
source install/setup.bash
```

### Run

```bash
# Run the node
ros2 run drone_offboard smart_follow_node

# Run with custom parameters
ros2 run drone_offboard smart_follow_node --ros-args \
    -p follow_dist:=8.0 \
    -p follow_height:=5.0 \
    -p terrain_follow_enable:=true \
    -p adaptive_distance_enable:=true
```

### Test

```bash
colcon test --packages-select drone_offboard
colcon test-result --verbose
```

## 📊 Comparison with AP_Follow

| Feature | AP_Follow | SmartFollowNode |
|---------|-----------|-----------------|
| **Platform** | Native ArduPilot | ROS 2 DDS |
| **Data Source** | MAVLink (GLOBAL_POSITION_INT, FOLLOW_TARGET) | ROS 2 Topics (NavSatFix) |
| **Velocity Filter** | None (raw MAVLink) | Alpha-Beta Filter |
| **Lateral FF** | ❌ | ✅ `ff_lateral_gain` |
| **Centripetal** | ❌ | ✅ `ff_centripetal_gain` |
| **Adaptive Distance** | ❌ | ✅ Speed-based |
| **Terrain Follow** | ✅ (FOLL_ALT_TYPE) | ✅ `terrain_follow_enable` |
| **Kinematic Shaping** | ✅ Separate NE/D/H | ✅ Separate NE/D/H |
| **Control Rate** | ~50Hz (internal) | 50Hz (configurable) |

## 🔬 Algorithm Details

### 1. Alpha-Beta Filter

The Alpha-Beta filter provides smooth velocity estimation from noisy GPS data:

```
x_pred = x + v × dt
x_new = x_pred + α × (measurement - x_pred)
v_new = v + (β × (measurement - x_pred)) / dt
```

- `α` (alpha): Trust in measurement (higher = more reactive)
- `β` (beta): Velocity update rate (lower = smoother)

### 2. Kinematic Prediction

Position prediction using second-order kinematics:

$$P_{future} = P_{current} + V \cdot t_{pred} + \frac{1}{2} A \cdot t_{pred}^2$$

### 3. Feed-Forward Control

```cpp
cmd.vel_forward = Kp × err_fwd + ff_vel_fwd;
cmd.vel_left = Kp × err_left + (ff_lateral_gain × ff_vel_left) + ff_centripetal;
```

Where:
- `ff_centripetal = target_speed × target_yaw_rate × centripetal_gain`

### 4. 2-Stage Input Shaping (from AP_Follow)

```
Stage 1: Jerk Limiting
    desired_acc = (desired_vel - last_vel) / dt
    delta_acc = clamp(desired_acc - last_acc, ±jerk_max × dt)
    shaped_acc = last_acc + delta_acc

Stage 2: Acceleration Limiting  
    shaped_acc = clamp(shaped_acc, ±accel_max)
    desired_vel = last_vel + shaped_acc × dt
```

## 📁 File Structure

```
drone_offboard/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE
├── include/
│   └── drone_offboard/
│       └── smart_follow_node.hpp
├── src/
│   ├── smart_follow_main.cpp
│   └── smart_follow_node.cpp
└── test/
    └── test_smart_follow_node.cpp
```

## 🛡️ Safety Features

1. **Flight Mode Check**: Only sends commands when in GUIDED mode and armed
2. **Target Timeout**: Stops drone if target GPS lost for > timeout seconds
3. **Estimate Validation**: Resets filters if velocity/acceleration unrealistic
4. **Velocity Clamps**: Hard limits on all velocity commands
5. **Command History Reset**: Prevents jerks when re-engaging after mode change

## 📝 License

Apache License 2.0

## 👤 Author

Quang Dung Ho (hoquangdung2003@gmail.com)

## 🔗 References

- [ArduPilot AP_Follow Source Code](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Follow/AP_Follow.cpp)
- [ArduPilot ROS 2 DDS Documentation](https://ardupilot.org/dev/docs/ros2.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
