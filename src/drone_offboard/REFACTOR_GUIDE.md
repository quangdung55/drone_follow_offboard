# 🚀 Core Architecture Refactor - Complete Guide

## 📋 What Was Done

This refactor separates **algorithm core** from **ROS adapter**, following professional software engineering practices used in ArduPilot, PX4, and commercial drone systems.

### Before Refactor
```
smart_follow_node.cpp (1500+ lines)
├── ROS callbacks (GPS, pose, gimbal)
├── GPS processing (jitter, filter, velocity)
├── Control algorithms (sqrt controller, shaping)
├── Yaw control (gimbal + heading)
├── Safety checks
└── ROS publishing

❌ Problems:
- Cannot test without ROS
- Cannot reuse in PX4/firmware
- Hard to maintain (everything mixed)
- 90% of code is algorithms, not ROS
```

### After Refactor
```
core/ (NO ROS DEPENDENCY!)
├── types/          (State definitions)
├── estimator/      (GPS processing)
├── control/        (Follow + Yaw controllers)
└── safety/         (Validators)

ros2/
└── smart_follow_node.cpp (400 lines - just adapter)
    ├── ROS pub/sub
    ├── Message conversion
    └── Calls core

✅ Benefits:
- Pure C++ unit tests
- Reusable in any project
- Clear separation
- Professional architecture
```

---

## 📁 File Structure

```
drone_offboard/
├── include/drone_offboard/
│   ├── core/                            # ← NEW: Algorithm core
│   │   ├── types/
│   │   │   ├── drone_state.hpp
│   │   │   ├── target_state.hpp
│   │   │   ├── nav_command.hpp
│   │   │   ├── common.hpp
│   │   │   └── parameters.hpp
│   │   ├── estimator/
│   │   │   └── target_estimator.hpp
│   │   ├── control/
│   │   │   ├── follow_controller.hpp
│   │   │   └── yaw_controller.hpp
│   │   └── safety/
│   │       ├── estimate_validator.hpp
│   │       ├── velocity_limiter.hpp
│   │       └── altitude_safety.hpp
│   └── smart_follow_node.hpp            # Original (for comparison)
│
├── src/
│   ├── core/                            # ← NEW: Algorithm implementation
│   │   ├── estimator/
│   │   │   └── target_estimator.cpp
│   │   ├── control/
│   │   │   ├── follow_controller.cpp
│   │   │   └── yaw_controller.cpp
│   │   └── safety.cpp
│   ├── smart_follow_node.cpp            # Original (legacy)
│   └── smart_follow_node_refactored_example.cpp  # ← NEW: Refactored node
│
├── test/
│   └── test_follow_controller_example.cpp  # ← NEW: Pure C++ unit tests
│
├── REFACTOR_SUMMARY.md                  # ← This guide
├── CMakeLists_REFACTORED_EXAMPLE.txt    # ← Build example
└── CMakeLists.txt                       # Original
```

---

## 🎯 Core Components

### 1. **TargetEstimator** - GPS Processing

**Extracted from**: `cb_target_gps()` (~200 lines → ~15 lines in callback)

**What it does**:
- Jitter correction (network delay compensation)
- DT validation (reject invalid timestamps)
- GPS to ENU conversion
- Alpha-Beta filtering (noise reduction)
- Velocity estimation (finite differences)
- Acceleration estimation (low-pass filtered)
- Heading calculation (from velocity vector)

**Usage**:
```cpp
#include "drone_offboard/core/estimator/target_estimator.hpp"

// Setup
EstimatorParams params;
params.filter_enable = true;
params.filter_alpha = 0.7;
params.filter_beta = 0.1;
params.timeout_sec = 3.0;

OriginManager origin;
origin.set_origin(lat, lon, alt);

TargetEstimator estimator(params, origin);

// Update (in ROS callback)
GPSMeasurement meas;
meas.lat = msg->latitude;
meas.lon = msg->longitude;
meas.alt = msg->altitude;
meas.timestamp_sec = time_sec;

if (estimator.update(meas, local_time_sec)) {
    const TargetState& state = estimator.state();
    // Use state.pos_enu, state.vel_enu, etc.
}
```

---

### 2. **FollowController** - Position Control

**Extracted from**: `calculate_kinematics()` (~150 lines → ~5 lines)

**What it does**:
- ArduPilot-style sqrt controller (smooth position tracking)
- Velocity feedforward (anti-lag)
- Kinematic shaping (jerk/acceleration limits)
- Vertical control (altitude hold)
- Body frame transformation

**Usage**:
```cpp
#include "drone_offboard/core/control/follow_controller.hpp"

// Setup
FollowControlParams params;
params.kp_pos = 0.1;
params.follow_height = 3.0;
params.accel_max_ne = 2.5;
params.jerk_max_ne = 5.0;

FollowController controller(params);

// Update (in control loop)
NavCommand cmd = controller.update(drone_state, target_state, dt);
// cmd.vel_forward, cmd.vel_left, cmd.vel_up
```

---

### 3. **YawController** - Heading Control

**Extracted from**: `calculate_yaw_rate()` (~50 lines → ~3 lines)

**What it does**:
- Gimbal tracking (reactive - keep target in camera view)
- Heading follow (predictive - track target direction)
- Mode blending (configurable mix)
- Jerk-limited output

**Usage**:
```cpp
#include "drone_offboard/core/control/yaw_controller.hpp"

// Setup
YawControlParams params;
params.kp_yaw = 0.05;
params.gimbal_deadzone = 5.0;
params.heading_blend_enable = true;
params.heading_blend_weight = 0.3;

YawController controller(params);

// Update gimbal
GimbalMeasurement gimbal;
gimbal.pan_error_deg = -10.0;
gimbal.timestamp_sec = current_time;
controller.update_gimbal(gimbal);

// Calculate yaw rate
double yaw_rate = controller.update(drone, target, dt, current_time_sec);
```

---

### 4. **Safety Validators**

**Extracted from**: Various safety checks in node

**Components**:
- `EstimateValidator` - Detects GPS glitches, filter divergence
- `VelocityLimiter` - Clamps commands to safe limits
- `AltitudeSafety` - Checks altitude limits, provides emergency descent

**Usage**:
```cpp
#include "drone_offboard/core/safety/velocity_limiter.hpp"
#include "drone_offboard/core/safety/altitude_safety.hpp"

VelocityLimiter limiter;
AltitudeSafety altitude_check;

// Check altitude
if (!altitude_check.is_safe(drone_state)) {
    NavCommand emergency = altitude_check.get_emergency_descent_command();
    publish(emergency);
    return;
}

// Clamp velocities
auto result = limiter.check(cmd.vel_forward, cmd.vel_left, cmd.vel_up, cmd.yaw_rate);
if (!result.passed) {
    WARN("Velocity exceeded: %s", result.reason.c_str());
}
limiter.clamp(cmd.vel_forward, cmd.vel_left, cmd.vel_up, cmd.yaw_rate);
```

---

## 🧪 Testing (Pure C++ - NO ROS!)

### Unit Test Example

```cpp
#include <gtest/gtest.h>
#include "drone_offboard/core/control/follow_controller.hpp"

TEST(FollowController, MoveForwardWhenTargetAhead) {
    FollowControlParams params;
    params.kp_pos = 0.1;
    params.follow_height = 3.0;
    
    FollowController controller(params);
    
    DroneState drone;
    drone.pos_enu << 0, 0, 0;
    drone.yaw = 0.0;
    
    TargetState target;
    target.pos_enu << 10, 0, 0;
    target.valid = true;
    
    NavCommand cmd = controller.update(drone, target, 0.02);
    
    EXPECT_GT(cmd.vel_forward, 0.0);  // Should move forward
}
```

### Run Tests

```bash
# Build with tests
colcon build --packages-select drone_offboard

# Run tests
colcon test --packages-select drone_offboard
colcon test-result --verbose
```

---

## 🔧 Integration Steps

### Step 1: Update CMakeLists.txt

See `CMakeLists_REFACTORED_EXAMPLE.txt` for complete example.

Key changes:
```cmake
# Add core library (NO ROS!)
add_library(drone_follow_core STATIC
  src/core/estimator/target_estimator.cpp
  src/core/control/follow_controller.cpp
  src/core/control/yaw_controller.cpp
  src/core/safety.cpp
)

target_link_libraries(drone_follow_core PUBLIC Eigen3::Eigen)

# ROS node uses core
add_executable(smart_follow_node src/smart_follow_node_refactored.cpp)
target_link_libraries(smart_follow_node drone_follow_core)
ament_target_dependencies(smart_follow_node rclcpp sensor_msgs ...)
```

### Step 2: Refactor ROS Node

See `src/smart_follow_node_refactored_example.cpp` for complete example.

Key changes:
```cpp
// Before
class SmartFollowNode {
    void cb_target_gps(...) {
        // 200 lines of algorithm
    }
};

// After
class SmartFollowNode {
    TargetEstimator target_estimator_;
    
    void cb_target_gps(...) {
        GPSMeasurement meas = convert_ros_to_core(msg);
        if (target_estimator_.update(meas, local_time)) {
            target_state_ = target_estimator_.state();
        }
    }
};
```

### Step 3: Build and Test

```bash
# Clean build
rm -rf build install log
colcon build --packages-select drone_offboard

# Test
colcon test --packages-select drone_offboard

# Run refactored node
ros2 run drone_offboard smart_follow_node_refactored
```

---

## 📊 Metrics

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| ROS node lines | 1500+ | ~400 | **73% reduction** |
| Algorithm in core | 0% | ~1100 lines | **Fully reusable** |
| Unit testable | ❌ | ✅ | **100%** |
| ROS dependency | Everywhere | Node only | **Isolated** |
| Reusable in PX4 | ❌ | ✅ | **Yes** |

---

## 🎓 Learning Resources

### ArduPilot Reference
- [AP_Follow.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Follow/AP_Follow.cpp)
- [control.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_AttitudeControl/control.cpp)

### PX4 Reference
- [PositionControl](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/flight_tasks)
- [Pure C++ architecture](https://docs.px4.io/main/en/middleware/)

---

## ✅ Checklist

- [x] Core types created (NO ROS dependency)
- [x] TargetEstimator implemented
- [x] FollowController implemented
- [x] YawController implemented
- [x] Safety validators implemented
- [x] Example ROS node created
- [x] Unit test example created
- [x] CMakeLists example created
- [x] Documentation complete
- [ ] Verify build (user to do)
- [ ] Run unit tests (user to do)
- [ ] Test refactored node (user to do)

---

## 🚦 Next Steps

1. **Compile core library**:
   ```bash
   colcon build --packages-select drone_offboard
   ```

2. **Run unit tests**:
   ```bash
   colcon test --packages-select drone_offboard
   ```

3. **Test refactored node**:
   ```bash
   ros2 run drone_offboard smart_follow_node_refactored
   ```

4. **Compare with original**:
   - Run both nodes side-by-side
   - Verify identical behavior
   - Check performance

5. **Migrate fully**:
   - Once verified, replace original node
   - Remove legacy code
   - Update documentation

---

## 📞 Support

If you encounter issues:
1. Check that core compiles without ROS
2. Verify include paths in CMakeLists.txt
3. Check that ap_control.hpp is accessible
4. Ensure Eigen3 is installed

---

**Status**: ✅ **Architecture Complete - Ready for Integration**

**Author**: AI Assistant  
**Date**: 2026-02-07
