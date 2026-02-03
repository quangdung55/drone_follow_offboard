# Jitter Correction for Smart Follow

## Tổng Quan

**JitterCorrection** là thuật toán xử lý timing jitter khi nhận GPS timestamp từ thiết bị remote qua mạng (WiFi/4G/5G). Được port từ ArduPilot `AP_RTC/JitterCorrection`.

## Vấn Đề Cần Giải Quyết

### Network Jitter Example:
```
Remote Device (GPS)    Network (4G)      Local System (Drone)
      |                     |                    |
      |--[t=100ms]--------->|------150ms-------->| Arrive: 250ms
      |--[t=110ms]--------->|------120ms-------->| Arrive: 230ms (!)
      |--[t=120ms]--------->|------180ms-------->| Arrive: 300ms
      
❌ Message t=110ms arrives BEFORE t=100ms message!
❌ Không thể dùng arrival time trực tiếp
❌ Velocity/acceleration calculation sẽ bị sai lệch nghiêm trọng
```

### Hậu Quả Nếu Không Xử Lý:
- ❌ Vận tốc nhảy giật (velocity spikes)
- ❌ Gia tốc không chính xác
- ❌ Drone bay giật cục
- ❌ Position estimate không smooth

## Thuật Toán

### Nguyên Lý Hoạt Động:

```cpp
// 1. Tính transport lag hiện tại
int64_t diff_us = local_time - offboard_time;

// 2. Nếu message từ "future" → điều chỉnh offset (chọn min lag)
if (diff_us < link_offset_usec) {
    link_offset_usec = diff_us;  // Track minimum lag
}

// 3. Estimate corrected timestamp
int64_t estimate_us = offboard_time + link_offset_usec;

// 4. Clamp outliers
if (estimate_us + max_lag < local_time) {
    estimate_us = local_time - max_lag;  // Too old
}

// 5. Convergence (xử lý clock drift)
// Thu thập min_sample qua N loops → update offset
```

### Giả Định:
1. **Message không thể từ tương lai** (local time domain)
2. **Message không cũ hơn max_lag_ms** (default: 500ms)

## Cách Sử Dụng

### 1. Enable/Disable qua Parameters

```yaml
# config/smart_follow_params.yaml
smart_follow_node:
  ros__parameters:
    jitter_correction_enable: true   # Enable/disable
    jitter_max_lag_ms: 500           # Max transport lag (ms)
    jitter_convergence_loops: 100    # Convergence samples
```

### 2. Khi Nào Nên Enable?

✅ **Enable khi:**
- GPS từ app điện thoại qua 4G/5G
- GPS qua WiFi không ổn định
- MAVLink telemetry qua unreliable network
- Thấy velocity/acceleration nhảy giật

❌ **Disable khi:**
- GPS từ onboard module (trực tiếp UART/I2C)
- Wired connection ổn định
- Low latency network (<10ms jitter)
- Timestamp đã được filter ở source

### 3. Tuning Parameters

#### `jitter_max_lag_ms` (Default: 500ms)
- **Mục đích:** Giới hạn độ trễ tối đa cho phép
- **Tuning:**
  - 4G/5G: 500ms - 1000ms
  - WiFi: 200ms - 500ms
  - Wired: 50ms - 100ms
- **Nếu quá nhỏ:** Sẽ clamp messages hợp lệ
- **Nếu quá lớn:** Outliers không được filter

#### `jitter_convergence_loops` (Default: 100)
- **Mục đích:** Số samples để hội tụ về min lag
- **Tuning:**
  - GPS 1Hz: 100 samples = 100 seconds
  - GPS 5Hz: 100 samples = 20 seconds
  - GPS 10Hz: 100 samples = 10 seconds
- **Nếu quá nhỏ:** Hội tụ nhanh nhưng nhạy với noise
- **Nếu quá lớn:** Chậm adapt với clock drift

## Diagnostics

### Check Jitter Correction Status:

```bash
# Xem log khi node start
ros2 run drone_offboard smart_follow_node

# Output:
# Jitter Correction: ENABLED (Max Lag=500ms, Convergence=100 samples)
```

### Monitor Link Offset:

```cpp
// Trong code (debug):
int64_t offset_usec = jitter_correction_.get_link_offset_usec();
RCLCPP_INFO(get_logger(), "Link offset: %ld us", offset_usec);
```

**Offset giá trị điển hình:**
- Local GPS: 0 - 10,000 µs (0-10ms)
- WiFi: 10,000 - 100,000 µs (10-100ms)
- 4G: 50,000 - 500,000 µs (50-500ms)
- 5G: 20,000 - 200,000 µs (20-200ms)

## Implementation Details

### GPS Message Flow:

```cpp
void SmartFollowNode::cb_target_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // 1. Extract offboard timestamp (from GPS sensor)
    uint64_t offboard_usec = msg->header.stamp.sec * 1000000ULL 
                           + msg->header.stamp.nanosec / 1000ULL;
    
    // 2. Get local timestamp (arrival time)
    uint64_t local_usec = this->now().nanoseconds() / 1000ULL;
    
    // 3. Apply jitter correction
    uint64_t corrected_usec = jitter_correction_.correct_offboard_timestamp_usec(
        offboard_usec,  // FROM sensor
        local_usec      // WHEN received
    );
    
    // 4. Use corrected timestamp for dt calculation
    double now_sec = corrected_usec / 1000000.0;
    double dt = now_sec - target_.last_update.seconds();
    
    // 5. Calculate velocity/acceleration with stable dt
    velocity = (pos_new - pos_old) / dt;  // ✅ Smooth, no spikes
}
```

### Timestamp Requirements:

**⚠️ QUAN TRỌNG:** GPS message **PHẢI** có timestamp hợp lệ trong header!

```cpp
// Good: GPS message với timestamp
msg->header.stamp.sec = 12345;
msg->header.stamp.nanosec = 678900000;

// Bad: Empty timestamp
msg->header.stamp.sec = 0;
msg->header.stamp.nanosec = 0;
// → JitterCorrection fallback to local time (no correction)
```

## Performance Impact

| Metric | Impact |
|--------|--------|
| **CPU Usage** | Negligible (<0.1%) |
| **Memory** | ~40 bytes per instance |
| **Latency** | <1µs per call |
| **Overhead** | Minimal (simple min tracking) |

## Testing

### Test Jitter Correction:

```bash
# 1. Run node với jitter correction enabled
ros2 run drone_offboard smart_follow_node --ros-args \
  -p jitter_correction_enable:=true \
  -p jitter_max_lag_ms:=500

# 2. Publish GPS với jitter
ros2 topic pub /target/gps sensor_msgs/msg/NavSatFix "..."

# 3. Monitor velocity smoothness
ros2 topic echo /ap/cmd_vel
```

### Expected Results:

**Với Jitter Correction:**
```
Velocity: 1.2 m/s
Velocity: 1.3 m/s
Velocity: 1.25 m/s
✅ Smooth transitions
```

**Không Jitter Correction:**
```
Velocity: 1.2 m/s
Velocity: 5.8 m/s  ← Spike!
Velocity: 0.3 m/s  ← Spike!
❌ Erratic behavior
```

## Troubleshooting

### Issue 1: Velocity vẫn nhảy giật

**Nguyên nhân:**
- GPS message không có timestamp hợp lệ
- `jitter_max_lag_ms` quá nhỏ
- Source timestamp không stable

**Giải pháp:**
```yaml
# Tăng max lag
jitter_max_lag_ms: 1000

# Hoặc disable nếu không hiệu quả
jitter_correction_enable: false
```

### Issue 2: Convergence quá chậm

**Nguyên nhân:**
- `jitter_convergence_loops` quá lớn
- GPS update rate thấp

**Giải pháp:**
```yaml
# Giảm convergence samples
jitter_convergence_loops: 50  # Với GPS 5Hz = 10 seconds
```

### Issue 3: Offset không hội tụ

**Nguyên nhân:**
- Clock drift giữa remote và local quá lớn
- Timestamp không đồng bộ

**Giải pháp:**
- Kiểm tra clock sync (NTP)
- Sử dụng ROS time sync
- Tăng convergence window

## References

- **ArduPilot Source:** `libraries/AP_RTC/JitterCorrection.h`
- **AP_Follow Usage:** `libraries/AP_Follow/AP_Follow.cpp:775`
- **Algorithm Paper:** "Minimum-Lag Transport Estimation for Time-Jittered Streams"

## License

GPL-3.0 (same as ArduPilot)
