# 🔄 Migration Quick Guide

## Current Status

File `smart_follow_node.cpp` đã được **marked for refactoring** với:

✅ **Documentation complete** - All refactor opportunities documented
✅ **Core components ready** - See `core/` directory  
✅ **Example implementation** - See `smart_follow_node_refactored_example.cpp`
✅ **Unit tests ready** - See `test/test_follow_controller_example.cpp`

## 🔍 Find Refactor Opportunities

Search trong file `smart_follow_node.cpp` cho:
```
🔄 REFACTOR OPPORTUNITY
```

Sẽ thấy 3 locations chính:

### 1. **Target GPS Processing** (Line ~400)
```cpp
void SmartFollowNode::cb_target_gps(...) {
    // 🔄 REFACTOR OPPORTUNITY
    // Current: ~200 lines
    // After:   ~15 lines
}
```

**Refactored version:**
```cpp
void SmartFollowNode::cb_target_gps(...) {
    GPSMeasurement meas;
    meas.lat = msg->latitude;
    meas.lon = msg->longitude;
    meas.alt = msg->altitude;
    meas.timestamp_sec = corrected_time;
    
    if (target_estimator_.update(meas, local_time)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_state_ = target_estimator_.state();
    }
}
```

### 2. **Position Control** (Line ~830)
```cpp
NavCommand SmartFollowNode::calculate_kinematics(...) {
    // 🔄 REFACTOR OPPORTUNITY
    // Current: ~150 lines
    // After:   ~5 lines
}
```

**Refactored version:**
```cpp
NavCommand SmartFollowNode::calculate_kinematics(...) {
    return follow_controller_.update(drone, target, dt);
}
```

### 3. **Yaw Control** (Line ~1000)
```cpp
double SmartFollowNode::calculate_yaw_rate(...) {
    // 🔄 REFACTOR OPPORTUNITY
    // Current: ~50 lines
    // After:   ~3 lines
}
```

**Refactored version:**
```cpp
double SmartFollowNode::calculate_yaw_rate(...) {
    return yaw_controller_.update(drone, target, dt, this->now().seconds());
}
```

---

## 📊 Impact

| Metric | Before | After | Change |
|--------|--------|-------|--------|
| ROS node lines | 1421 | ~400 | **-72%** |
| Algorithm in callbacks | 100% | 0% | **Extracted** |
| Unit testable | ❌ | ✅ | **100%** |
| Reusable (PX4/firmware) | ❌ | ✅ | **Yes** |

---

## 🚀 Quick Migration (One Function)

### Step 1: Uncomment Core Includes
In `smart_follow_node.cpp` line ~25:
```cpp
// 🔧 Uncomment these:
#include "drone_offboard/core/estimator/target_estimator.hpp"
using namespace drone_follow::core;
```

### Step 2: Add Member Variable
In `smart_follow_node.hpp`, add:
```cpp
// In private section
std::unique_ptr<TargetEstimator> target_estimator_;
```

### Step 3: Initialize in Constructor
In `SmartFollowNode::SmartFollowNode()`:
```cpp
// After setup_parameters()
EstimatorParams est_params;
est_params.filter_enable = param_filter_enable_;
est_params.filter_alpha = param_filter_alpha_;
// ... load other params
target_estimator_ = std::make_unique<TargetEstimator>(est_params, origin_manager_);
```

### Step 4: Refactor Callback
Replace `cb_target_gps()` body with refactored version (see above).

### Step 5: Test
```bash
colcon build --packages-select drone_offboard
ros2 run drone_offboard smart_follow_node
# Verify behavior matches original
```

---

## 📚 Full Documentation

See **[REFACTOR_GUIDE.md](REFACTOR_GUIDE.md)** for:
- Complete implementation details
- Unit test examples
- CMakeLists.txt configuration
- All core component usage
- Troubleshooting

---

## ✅ Verification Checklist

After migration:
- [ ] Node compiles successfully
- [ ] Behavior matches original (compare flight logs)
- [ ] No ROS errors/warnings
- [ ] Unit tests pass
- [ ] Core can compile without ROS (test separately)

---

**Status**: 📝 **Documentation Complete - Ready for Migration**

**Next**: Choose one function to migrate (recommend starting with `calculate_yaw_rate()` - simplest)
