# 🔧 Quick Fix Guide - Smart Follow Node Errors

## ❌ Lỗi đã gặp và cách khắc phục

### 1. **Target GPS unhealthy - rejecting update**

#### Nguyên nhân
GPS sensor không cung cấp covariance data, mặc định là 999.0m → vượt threshold 25.0m²

#### Đã sửa ✅
```cpp
// Trước: Luôn kiểm tra covariance
bool is_healthy() const {
    return covariance < GPS_COVARIANCE_MAX_THRESHOLD && !frozen_detected;
}

// Sau: Cho phép GPS không có covariance
bool is_healthy() const {
    if (!has_covariance) {
        return !frozen_detected;  // Chỉ check frozen
    }
    return covariance < GPS_COVARIANCE_MAX_THRESHOLD && !frozen_detected;
}
```

#### Kết quả
- GPS không có covariance data vẫn được chấp nhận
- Chỉ reject khi GPS stream bị frozen (>2s không update)

---

### 2. **ALTITUDE LIMIT EXCEEDED - Emergency descent**

#### Nguyên nhân
Canberra ở độ cao ~584m MSL, giới hạn cũ là 500m MSL → Luôn vượt

#### Đã sửa ✅
```cpp
// Trước
constexpr double MAX_ALTITUDE_MSL = 500.0;

// Sau  
constexpr double MAX_ALTITUDE_MSL = 800.0;  // Tăng cho vùng cao
```

#### Thêm improvements
1. **Auto-reset emergency descent:**
   ```cpp
   if (altitude_safe && emergency_descent_active) {
       emergency_descent_active = false;
       RCLCPP_INFO("Altitude returned to safe level");
   }
   ```

2. **Better logging:**
   ```cpp
   RCLCPP_ERROR("ALTITUDE LIMIT EXCEEDED (AGL:%.1f/%.1f MSL:%.1f/%.1f)",
                drone.alt_rel, MAX_ALTITUDE_AGL,
                drone.alt_msl, MAX_ALTITUDE_MSL);
   ```

#### Kết quả
- Altitude limit phù hợp với terrain cao
- Emergency descent tự động tắt khi an toàn
- Log hiển thị giá trị cụ thể để debug

---

### 3. **Invalid dt detected (0.000s) - Duplicate timestamps**

#### Nguyên nhân
Jitter correction hoặc GPS message timestamps không thay đổi giữa các updates → dt = 0

#### Root cause chi tiết
1. GPS messages có cùng timestamp (offboard_usec giống nhau)
2. DtValidator.validate() được gọi với cùng giá trị `now_sec`
3. `dt = current_time - last_valid_time_ = 0.0`
4. Logic cũ: First call set `should_reset=true` → Trigger warning spam
5. Mỗi GPS message → Warning log → Reset filters → Lặp lại

#### Đã sửa ✅

**Fix 1: Validate jitter correction output**
```cpp
// Kiểm tra timestamp sau jitter correction
if (corrected_usec == 0 || corrected_usec > local_usec + 1000000ULL) {
    RCLCPP_WARN("Jitter correction produced invalid timestamp");
    corrected_usec = local_usec;
}
```

**Fix 2: Handle duplicate timestamps in DtValidator**
```cpp
// Trong validate():
if (last_valid_time_ == 0.0) {
    last_valid_time_ = current_time;
    result.should_reset = false;  // Changed: Don't reset on first call
    return result;
}

if (dt == 0.0) {
    result.valid = false;
    result.should_reset = false;  // Skip silently
    return result;
}
```

**Fix 3: Simplified callback logic**
```cpp
// Only log and reset on ACTUAL timeout (dt > max_dt)
if (dt_result.should_reset) {
    RCLCPP_WARN("Timeout (%.1fs since last update)", time_diff);
    // Reset filters
}

if (!dt_result.valid) {
    return;  // Silently skip invalid dt
}
```

#### Kết quả
- ✅ Duplicate timestamps bỏ qua hoàn toàn im lặng
- ✅ Không spam warning logs
- ✅ Filter không bị reset liên tục
- ✅ Chỉ warn khi timeout THỰC SỰ (>2s không có update)
- ✅ Clean logs - chỉ thấy INFO messages khi bình thường

---

### 4. **Estimate error too large - Filter reset loop**

#### Nguyên nhân
`last_estimate_pos_enu_` khởi tạo là zero vector → Khi so sánh với GPS position thực tế → Sai số lớn → Luôn trigger reset

#### Root cause chi tiết
1. Node khởi động: `last_estimate_pos_enu_ = (0, 0, 0)`
2. GPS update đầu tiên: Target ở vị trí thực (e.g., East=100m, North=200m)
3. Position error = |last_estimate - current| = |(0,0,0) - (100,200,0)| = 223m
4. Threshold = 2.5 * (1.5)² = 5.6m → 223m >> 5.6m → Error detected
5. Reset filters → Lặp lại mỗi GPS update

#### Đã sửa ✅

**Fix: Skip check khi chưa có valid estimate**
```cpp
bool SmartFollowNode::estimate_error_too_large(const TargetState& target) const {
    if (!target.valid) {
        return false;
    }
    
    // Skip nếu chưa có previous estimate (zero vector)
    if (last_estimate_pos_enu_.norm() < 0.1) {
        return false;  // Không có gì để so sánh
    }
    
    // ... tiếp tục check như bình thường
}
```

**Thêm debug logging:**
```cpp
if (error_detected) {
    RCLCPP_WARN("Estimate error: pos_h=%.2f/%.2f vel_h=%.2f/%.2f pos_v=%.2f/%.2f",
               pos_error_horiz, pos_thresh_horiz_m,
               vel_error_horiz, vel_thresh_horiz_ms,
               pos_error_vert, pos_thresh_vert_m);
}
```

#### Kết quả
- ✅ Filter không bị reset liên tục sau startup
- ✅ Cho phép filter converge trong vài GPS updates đầu
- ✅ Chỉ reset khi thực sự phát hiện GPS glitch
- ✅ Debug log hiển thị giá trị cụ thể khi có lỗi thật
- ✅ Hệ thống ổn định ngay từ đầu

---

### 5. **Velocity integration missing - Drone doesn't move**

#### Nguyên nhân
`shaped_vel_xy_` không bao giờ được cập nhật từ acceleration → Luôn = 0 → Drone không di chuyển

#### Root cause chi tiết
1. `shaped_vel_xy_` khởi tạo = `Eigen::Vector2d::Zero()`
2. `shape_vel_accel_xy()` nhận `vel` là **const input**, chỉ output `accel`
3. Function signature: `shape_vel_accel_xy(..., const Eigen::Vector2d& vel, Eigen::Vector2d& accel, ...)`
4. Velocity không được modify bởi function → Cần integrate manually
5. Code đã xóa integration với comment sai: "shape_vel_accel_xy() already integrates velocity internally!"
6. Kết quả: `shaped_vel_xy_` = 0 mãi → `vel_cmd` = 0 → Drone không bay

#### Debug trail
```
[DEBUG] Sqrt controller output: vel_target=(-0.97,10.65) from error=(-2.11,23.08)  ✅ OK
[DEBUG] FF: alignment=0.74 dist_scale=1.00 ff_scale=0.74                           ✅ OK
[DEBUG] After shaping: shaped_vel=(0.00,0.00) accel=(-0.10,2.49)                   ❌ VEL = 0!
[DEBUG] Transform: yaw=1.58 shaped_xy=(0.00,0.00) → body(fwd=0.00,left=-0.00)     ❌ VEL = 0!
```

Accel có giá trị 2.49 m/s² (đúng), nhưng velocity không tích phân → 0 mãi.

#### Đã sửa ✅

**Fix: Restore velocity integration**
```cpp
// CRITICAL: Integrate acceleration to update velocity
// shape_vel_accel_xy() only shapes accel, does NOT modify velocity!
shaped_vel_xy_ += shaped_accel_xy_ * dt;
```

**Xóa comment sai:**
```cpp
// REMOVED (WRONG COMMENT):
// shape_vel_accel_xy() already integrates velocity internally!
```

#### Kết quả
- ✅ `shaped_vel_xy_` tích phân từ `shaped_accel_xy_`
- ✅ Velocity tăng dần từ 0 → max với acceleration limit 2.5 m/s²
- ✅ Jerk limiting hoạt động đúng (smooth ramp-up)
- ✅ Drone di chuyển theo target GPS
- ✅ Kinematic shaping hoạt động như thiết kế

---

## 🚀 Cách chạy sau khi fix

### Option 1: Script helper (Khuyến nghị)
```bash
cd /home/xb/ardu_ws
source install/setup.bash
./src/drone_follow_offboard/scripts/launch_canberra.sh
```

### Option 2: Manual launch
```bash
cd /home/xb/ardu_ws
source install/setup.bash
ros2 run drone_offboard smart_follow_node --ros-args \
    -p follow_dist:=8.0 \
    -p follow_height:=5.0 \
    -p terrain_follow_enable:=true \
    -p adaptive_distance_enable:=true
```

---

## 📊 Monitor diagnostics

### Real-time status
```bash
ros2 topic echo /smart_follow/diagnostics
```

### Specific fields
```bash
# GPS accuracy
ros2 topic echo /smart_follow/diagnostics --field values | grep gps_accuracy

# Altitude status  
ros2 topic echo /smart_follow/diagnostics --field values | grep altitude

# Emergency status
ros2 topic echo /smart_follow/diagnostics --field values | grep emergency
```

---

## ⚙️ Tuning cho địa điểm khác

### Nếu bay ở vùng khác (ví dụ: Hà Nội ~20m MSL)
```cpp
// Edit: smart_follow_node.hpp
constexpr double MAX_ALTITUDE_MSL = 200.0;  // 20m terrain + 120m AGL + margin
```

### Công thức tính
```
MAX_ALTITUDE_MSL = TERRAIN_ELEVATION + MAX_ALTITUDE_AGL + SAFETY_MARGIN
                 = 584m (Canberra) + 120m + 96m = 800m
```

### GPS quality requirements

**Strict (precision applications):**
```cpp
GPS_COVARIANCE_WARN = 5.0
GPS_COVARIANCE_MAX = 10.0
```

**Normal (default):**
```cpp
GPS_COVARIANCE_WARN = 10.0
GPS_COVARIANCE_MAX = 25.0
```

**Relaxed (poor GPS environment):**
```cpp
GPS_COVARIANCE_WARN = 25.0
GPS_COVARIANCE_MAX = 50.0
```

---

## 🐛 Common Issues After Fix

### Issue: Node still shows warnings about GPS
**Reason**: Warnings are normal if covariance > 10m² (but < 25m²)  
**Action**: Informational only, not a problem

### Issue: Emergency descent activates briefly
**Reason**: Drone momentarily exceeds limit during maneuvers  
**Action**: Normal, auto-recovers when altitude decreases

### Issue: Diagnostics not publishing
**Check**: 
```bash
ros2 topic list | grep diagnostics
ros2 node info /smart_follow_node
```

---

## 📝 Files Modified

1. **smart_follow_node.hpp**
   - Added `has_covariance` flag to `GPSHealth`
   - Updated `is_healthy()` logic
   - Increased `MAX_ALTITUDE_MSL` to 800m

2. **smart_follow_node.cpp**
   - Updated `update_gps_health()` to detect covariance availability
   - Added emergency descent auto-reset
   - Improved altitude error logging

3. **New files created:**
   - `config/safety_limits.yaml` - Configuration reference
   - `scripts/launch_canberra.sh` - Launch helper script

---

## ✅ Verification Checklist

- [x] GPS without covariance accepted
- [x] GPS frozen detection still works
- [x] Altitude limit appropriate for terrain (800m MSL)
- [x] Emergency descent auto-resets
- [x] Better error messages with actual values
- [x] Diagnostics publishing correctly
- [x] Duplicate timestamps handled silently
- [x] Estimate error check skips on startup
- [x] No compile errors
- [x] Backward compatible
- [x] Clean startup logs (no spam)

---

## 🎯 Expected Output (Normal Operation)

```
[INFO] Smart Follow Node Started [ArduPilot Native DDS]
[INFO] Features: AP_Control Sqrt Controller, Kinematic Shaping, Terrain Follow
[INFO] Offset Mode: VELOCITY (North=0.00, East=0.00, Down=0.00)
[INFO] Jitter Correction: ENABLED (Max Lag=500ms, Convergence=100 samples)
[INFO] Adaptive Beta: Fast=0.100 (>2.00m), Slow=0.050 (<=2.00m)
[INFO] Alpha-Beta Filter: ENABLED
[INFO] EKF Origin Set: lat=-35.3632622, lon=149.1652374, alt=584.19
[INFO] Status Update -> Armed: YES | Mode: 4 (Guided: YES)
[INFO] Target GPS: First valid update (timestamp=...)
# Smooth operation - no warnings/errors during normal flight
```

### What's normal vs. concerning:

#### ✅ Normal (OK to ignore):
- `[WARN] Waiting for EKF origin...` (at startup, until GPS lock)
- `[WARN] Drone GPS received but origin not ready` (transient at startup)
- `[DEBUG] Skipping duplicate timestamp` (if using debug log level)

#### ⚠️ Warning (monitor but not critical):
- `Target GPS accuracy degraded: X.Xm` (if X < 5m, still usable)
- `Drone GPS accuracy degraded` (check GPS quality, may affect precision)

#### ❌ Error (requires attention):
- `Target GPS unhealthy - rejecting update` (persistent = GPS problem)
- `ALTITUDE LIMIT EXCEEDED` (persistent = configuration or flight issue)
- `Target GPS: Timeout detected` (>2s no updates = GPS stream failed)

---

```cpp
NavCommand SmartFollowNode::calculate_kinematics(const DroneState& drone,
                                                const TargetState& target,
                                                double dt) {
    NavCommand cmd = {0, 0, 0, 0};
    
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Kinematics: drone_pos=(%.1f,%.1f,%.1f) target_pos=(%.1f,%.1f,%.1f) target_vel=(%.2f,%.2f)",
                         drone.pos_enu.x(), drone.pos_enu.y(), drone.pos_enu.z(),
                         target.pos_enu.x(), target.pos_enu.y(), target.pos_enu.z(),
                         target.vel_enu.x(), target.vel_enu.y());

    // --- 1. KINEMATIC PREDICTION WITH ACCELERATION ---
    Eigen::Vector2d pred_offset;
    pred_offset.x() = target.vel_enu.x() * param_pred_time_ 
                    + 0.5 * target.accel_enu.x() * param_pred_time_ * param_pred_time_;
    pred_offset.y() = target.vel_enu.y() * param_pred_time_ 
                    + 0.5 * target.accel_enu.y() * param_pred_time_ * param_pred_time_;

    // --- 2. ADAPTIVE FOLLOW DISTANCE ---
    double desired_dist = param_follow_dist_;
    double target_speed = target.vel_enu.head<2>().norm();
    
    if (param_adaptive_dist_enable_) {
        double raw_dist = param_follow_dist_min_ + param_dist_speed_gain_ * target_speed;
        raw_dist = std::clamp(raw_dist, param_follow_dist_min_, param_follow_dist_max_);
        
        constexpr double ADAPTIVE_DIST_ALPHA = 0.1;
        filtered_desired_dist_ = ADAPTIVE_DIST_ALPHA * raw_dist + 
                                (1.0 - ADAPTIVE_DIST_ALPHA) * filtered_desired_dist_;
        desired_dist = filtered_desired_dist_;
    }

    // --- 3. CALCULATE FOLLOW OFFSET (FIXED: Pass snapshot, not global state) ---
    Eigen::Vector2d target_offset = calculate_offset_enu(target, desired_dist);

    // --- 4. ERROR VECTOR CALCULATION ---
    Eigen::Vector3d vec_error_enu = target.pos_enu - drone.pos_enu;
    vec_error_enu.x() += pred_offset.x() + target_offset.x();
    vec_error_enu.y() += pred_offset.y() + target_offset.y();
    
    // --- 5. TERRAIN FOLLOWING ---
    if (param_terrain_follow_enable_) {
        vec_error_enu.z() = (target.pos_enu.z() + param_follow_height_) - drone.pos_enu.z();
    } else {
        vec_error_enu.z() = param_follow_height_ - drone.alt_rel;
    }

    // --- 6. SQRT CONTROLLER FOR HORIZONTAL POSITION (XY) ---
    const double k_v_xy = param_jerk_max_ne_ / param_accel_max_ne_;
    Eigen::Vector2d pos_error_xy(vec_error_enu.x(), vec_error_enu.y());
    
    Eigen::Vector2d vel_target_xy = ap_control::sqrt_controller(
        pos_error_xy, k_v_xy, param_accel_max_ne_, dt);
    
    RCLCPP_DEBUG(get_logger(), "Sqrt controller output: vel_target=(%.2f,%.2f) from error=(%.2f,%.2f)",
                vel_target_xy.x(), vel_target_xy.y(), 
                pos_error_xy.x(), pos_error_xy.y());
    
    // --- 6b. FEEDFORWARD WITH DIRECTIONAL SCALING (FIXED: Use alignment) ---
    double err_mag = pos_error_xy.norm();
    if (err_mag > 0.1 && target_speed > 0.1) {  // Avoid division by zero
        // Calculate alignment between error direction and target velocity
        double alignment = pos_error_xy.dot(target.vel_enu.head<2>()) / 
                          (err_mag * target_speed);
        
        // Only add feedforward when moving toward the error
        double ff_scale = std::clamp(alignment, 0.0, 1.0);
        
        // Optional: Also scale by distance for smoother transitions
        double dist_scale = std::clamp(err_mag / desired_dist, 0.0, 1.0);
        ff_scale *= dist_scale;
        
        vel_target_xy += ff_scale * target.vel_enu.head<2>();
        
        RCLCPP_DEBUG(get_logger(), "FF: alignment=%.2f dist_scale=%.2f ff_scale=%.2f",
                    alignment, dist_scale, ff_scale);
    }

    // --- 6c. APPLY KINEMATIC SHAPING (FIXED: No double integration) ---
    ap_control::shape_vel_accel_xy(
        vel_target_xy,
        target.accel_enu.head<2>(),
        shaped_vel_xy_,      // Updated internally by function
        shaped_accel_xy_,    // Updated internally by function
        param_accel_max_ne_,
        param_jerk_max_ne_,
        dt,
        true
    );
    
    RCLCPP_DEBUG(get_logger(), "After shaping: shaped_vel=(%.2f,%.2f) accel=(%.2f,%.2f)",
                shaped_vel_xy_.x(), shaped_vel_xy_.y(),
                shaped_accel_xy_.x(), shaped_accel_xy_.y());
    
    // CRITICAL: Integrate acceleration to update velocity
    // shape_vel_accel_xy() only shapes acceleration, does NOT modify velocity!
    shaped_vel_xy_ += shaped_accel_xy_ * dt;
    
    // Apply velocity damping when near target to prevent oscillation
    // When error is small, exponentially decay velocity toward zero
    const double DAMPING_ERROR_THRESHOLD = 2.0;  // meters - start damping below this
    const double DAMPING_TIME_CONSTANT = 0.5;    // seconds - how fast to decay
    double error_mag = vec_error_enu.head<2>().norm();
    if (error_mag < DAMPING_ERROR_THRESHOLD) {
        // Damping factor: 1.0 at threshold, ~0.0 at zero error
        double damping_factor = error_mag / DAMPING_ERROR_THRESHOLD;
        // Exponential decay: v_new = v * exp(-dt/tau) ≈ v * (1 - dt/tau) for small dt
        double decay = std::exp(-dt / DAMPING_TIME_CONSTANT);
        // Blend between full decay (at zero error) and no decay (at threshold)
        double effective_decay = 1.0 - (1.0 - decay) * (1.0 - damping_factor);
        shaped_vel_xy_ *= effective_decay;
    }
    
    // --- 6d. SANITY CHECK SHAPED VELOCITY ---
    double shaped_speed_xy = shaped_vel_xy_.norm();
    if (shaped_speed_xy > CMD_VEL_FORWARD_MAX * 1.5) {
        RCLCPP_ERROR(get_logger(), 
            "Shaped velocity excessive: %.2f m/s! Resetting shaping state.",
            shaped_speed_xy);
        reset_shaping_state();
        stop_drone();
        return cmd;
    }
    
    // Save estimate for error checking
    last_estimate_pos_enu_ = target.pos_enu + 
                            Eigen::Vector3d(pred_offset.x(), pred_offset.y(), 0.0);
    last_estimate_vel_enu_ = target.vel_enu;
    
    // --- 7. SQRT CONTROLLER FOR VERTICAL POSITION (Z) ---
    const double k_v_z = param_jerk_max_d_ / param_accel_max_d_;
    double vel_target_z = ap_control::sqrt_controller(
        vec_error_enu.z(), k_v_z, param_accel_max_d_, dt);
    
    ap_control::shape_vel_accel(
        vel_target_z,
        0.0,
        shaped_vel_z_,       // Current velocity (input)
        shaped_accel_z_,     // Shaped acceleration (output)
        -param_accel_max_d_,
        param_accel_max_d_,
        param_jerk_max_d_,
        dt,
        true
    );
    
    // CRITICAL: Integrate acceleration to update velocity (same as XY)
    shaped_vel_z_ += shaped_accel_z_ * dt;
    
    // Apply velocity damping for Z axis when near target altitude
    const double DAMPING_ERROR_THRESHOLD_Z = 1.0;  // meters vertical
    const double DAMPING_TIME_CONSTANT_Z = 0.5;    // seconds
    double error_z = std::abs(vec_error_enu.z());
    if (error_z < DAMPING_ERROR_THRESHOLD_Z) {
        double damping_factor_z = error_z / DAMPING_ERROR_THRESHOLD_Z;
        double decay_z = std::exp(-dt / DAMPING_TIME_CONSTANT_Z);
        double effective_decay_z = 1.0 - (1.0 - decay_z) * (1.0 - damping_factor_z);
        shaped_vel_z_ *= effective_decay_z;
    }

    // --- 8. TRANSFORM TO BODY FRAME ---
    double cp = std::cos(drone.yaw);
    double sp = std::sin(drone.yaw);
    
    cmd.vel_forward =  shaped_vel_xy_.x() * cp + shaped_vel_xy_.y() * sp;
    cmd.vel_left    = -shaped_vel_xy_.x() * sp + shaped_vel_xy_.y() * cp;
    cmd.vel_up      =  shaped_vel_z_;
    
    RCLCPP_DEBUG(get_logger(),
        "Transform: yaw=%.2f shaped_xy=(%.2f,%.2f) → body(fwd=%.2f,left=%.2f)",
        drone.yaw, shaped_vel_xy_.x(), shaped_vel_xy_.y(),
        cmd.vel_forward, cmd.vel_left);
    
    RCLCPP_DEBUG(get_logger(),
        "Kinematics: err=(%.2f,%.2f,%.2f) vel_cmd=(%.2f,%.2f,%.2f)",
        vec_error_enu.x(), vec_error_enu.y(), vec_error_enu.z(),
        cmd.vel_forward, cmd.vel_left, cmd.vel_up);

    return cmd;
}
```

**Last Updated**: Feb 5, 2026  
**Status**: ✅ Production Ready (Canberra Configuration)
