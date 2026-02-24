# 🐛 Timestamp Debugging Guide

## Problem: Duplicate Timestamp Warnings

If you see:
```
[WARN] Target GPS: Timeout detected (now=X.XXXs, last=X.XXXs, diff=0.000s)
```

This indicates **duplicate timestamps** (not actual timeout).

---

## Root Cause Analysis

### Why does dt = 0 happen?

1. **GPS message timestamp unchanged**
   - Sender publishes multiple messages with same header.stamp
   - Common with simulated GPS or high-rate publishers

2. **Jitter correction converges to same value**
   - JitterCorrection algorithm stabilizes on a timestamp
   - Multiple messages corrected to same usec value

3. **Clock precision issues**
   - System clock resolution too low
   - Timestamps rounded to same value

---

## Debug Steps

### 1. Check GPS message timestamps
```bash
# Monitor raw GPS timestamps
ros2 topic echo /target/gps --field header.stamp

# Expected: Timestamps should increment
# sec: 1770265474
# nanosec: 324000000
# sec: 1770265474
# nanosec: 535000000  ← Different
```

### 2. Check message rate
```bash
ros2 topic hz /target/gps

# Expected: ~5-10 Hz typical
# If > 100 Hz: May have duplicate timestamps
```

### 3. Enable debug logging
```bash
ros2 run drone_offboard smart_follow_node --ros-args \
    --log-level smart_follow_node:=debug
```

Look for:
```
[DEBUG] Offboard timestamp invalid (0x...), using local time
[DEBUG] Jitter correction produced invalid timestamp
```

### 4. Disable jitter correction (test)
```bash
ros2 run drone_offboard smart_follow_node --ros-args \
    -p jitter_correction_enable:=false
```

If warnings disappear → Jitter correction issue  
If warnings persist → GPS message issue

---

## Solutions

### Quick Fix: Disable jitter correction
```bash
# In launch file or command line
-p jitter_correction_enable:=false
```

**Pros:** Eliminates duplicate timestamp issue  
**Cons:** May have less smooth velocity estimation with network jitter

### Proper Fix: Fix GPS publisher
```python
# If you control the GPS publisher
msg.header.stamp = self.get_clock().now().to_msg()  # Use ROS time
# NOT: msg.header.stamp.sec = fixed_value
```

### Code Fix (already implemented): ✅
```cpp
// DtValidator silently skips dt=0
if (dt == 0.0) {
    return {default_dt_, false, false};  // Skip silently
}

// Callback skips invalid dt
if (!dt_result.valid) {
    return;  // No warning spam
}
```

---

## Expected Behavior After Fix

### Normal operation (NO warnings):
```
[INFO] Smart Follow Node Started
[INFO] EKF Origin Set: lat=...
[INFO] Target GPS: First valid update received
# Silent operation - updates processed normally
```

### With duplicate timestamps (SILENT skip):
```
# GPS messages with dt=0 are silently discarded
# Only valid dt updates are processed
# No warning spam
```

### Actual timeout (WARNING shown):
```
[WARN] Target GPS: Timeout (2.5s since last update), resetting filters
# Only appears when GPS stream actually stops
```

---

## Verification

### Test 1: Normal GPS stream
```bash
# Run node
./scripts/launch_canberra.sh

# Expected: Clean startup, no repeated warnings
# Should see: "First valid update received"
# Then: Silent operation
```

### Test 2: Stopped GPS stream
```bash
# Stop GPS publisher
ros2 topic pub /target/gps ... # Stop publishing

# Expected after 2s:
# [WARN] Target GPS: Timeout (2.X s since last update)
```

### Test 3: High-rate GPS
```bash
# Publish GPS at 100Hz with same timestamp
# Expected: Updates silently skipped, no warnings
```

---

## Code Explanation

### DtValidator Logic Flow

```
validate(current_time)
    ↓
Is first call? (last_valid_time == 0)
    YES → Initialize, return valid=false, should_reset=false
    NO  → Continue
    ↓
Calculate dt = current - last
    ↓
dt == 0? (Duplicate timestamp)
    YES → Return valid=false, should_reset=false (SILENT SKIP)
    NO  → Continue
    ↓
dt < min_dt? (Too fast)
    YES → Return valid=false, should_reset=false
    NO  → Continue
    ↓
dt > max_dt? (Timeout)
    YES → Return valid=false, should_reset=TRUE (RESET & WARN)
    NO  → Continue
    ↓
VALID → Update last_valid_time, return valid=true
```

### Key Change
```cpp
// OLD: First call triggered reset warning
if (last_valid_time_ == 0.0) {
    result.should_reset = true;  // ❌ Caused spam
}

// NEW: First call silently skipped
if (last_valid_time_ == 0.0) {
    result.should_reset = false;  // ✅ Silent skip
}
```

---

## Performance Impact

- **CPU**: Negligible (<0.01% increase from extra checks)
- **Memory**: No change
- **Latency**: No change (skipped updates were already invalid)
- **Logs**: Dramatically reduced (no spam)

---

## Troubleshooting

### Still seeing warnings after update?

1. **Check colcon build succeeded**
   ```bash
   colcon build --packages-select drone_offboard
   # Should finish without errors
   ```

2. **Source workspace**
   ```bash
   source install/setup.bash
   ```

3. **Verify binary is updated**
   ```bash
   ls -lh install/drone_offboard/lib/drone_offboard/smart_follow_node
   # Check timestamp is recent
   ```

4. **Kill old processes**
   ```bash
   killall smart_follow_node
   # Then relaunch
   ```

---

**Last Updated**: Feb 5, 2026  
**Status**: ✅ Fixed - Duplicate timestamps handled silently
