# 🔍 Troubleshooting: Drone Not Following Target

## Symptom
`/ap/cmd_vel` shows all zeros → Drone không di chuyển

## Quick Checks

### 1. Chạy node với debug logging
```bash
cd /home/xb/ardu_ws
source install/setup.bash
./src/drone_follow_offboard/scripts/launch_debug.sh
```

### 2. Kiểm tra target GPS có đến không
```bash
# Terminal mới
ros2 topic hz /target/gps
# Expected: ~5 Hz (tùy GPS update rate)
```

### 3. Xem target GPS data
```bash
ros2 topic echo /target/gps --field latitude,longitude,altitude
# Phải có giá trị thay đổi khi target di chuyển
```

### 4. Kiểm tra drone GPS
```bash
ros2 topic echo /ap/navsat --field latitude,longitude,altitude
```

### 5. Xem diagnostics
```bash
ros2 topic echo /smart_follow/diagnostics --field message,level
# level=0: OK
# level=1: WARN
# level=2: ERROR
```

## Common Causes & Solutions

### ❌ Cause 1: Target không di chuyển
**Symptom:** Target velocity = 0
```
Kinematics: target_vel=(0.00,0.00)
CMD: All zeros (on target or no movement needed)
```

**Why:** Offset mode VELOCITY → Offset = -vel/|vel| * dist
- Nếu vel=0 → offset=0 → Drone đã ở đúng vị trí target

**Solution:**
```bash
# Option 1: Đổi sang NED offset mode
ros2 param set /smart_follow_node offset_type 0
ros2 param set /smart_follow_node offset_north -8.0  # Follow 8m phía sau (South)

# Option 2: Đổi sang RELATIVE offset mode  
ros2 param set /smart_follow_node offset_type 1
ros2 param set /smart_follow_node offset_north -8.0
```

---

### ❌ Cause 2: Target velocity chưa được estimate
**Symptom:** GPS updates đầu tiên chưa có đủ data tính velocity
```
Target GPS: First valid update received
[DEBUG] Kinematics: target_vel=(0.00,0.00)
```

**Why:** Cần ít nhất 2 GPS updates để tính vận tốc (v = Δpos/Δt)

**Solution:** Đợi thêm 1-2 GPS updates (0.2-0.4s)

---

### ❌ Cause 3: Target GPS timeout
**Symptom:**
```
[WARN] Target timeout: 3.5s > 3.0s
```

**Solution:** Kiểm tra `/target/gps` topic có publish không

---

### ❌ Cause 4: Drone không ở chế độ Guided
**Symptom:**
```
Status Update -> Armed: YES | Mode: X (Guided: NO)
```

**Solution:** Set mode GUIDED trong ArduPilot

---

### ❌ Cause 5: Position error quá nhỏ
**Symptom:** Drone đã ở gần target
```
[DEBUG] Kinematics: drone_pos=(100.0,200.0) target_pos=(100.5,200.5)
CMD: All zeros
```

**Solution:** Di chuyển target xa hơn (>1m)

---

### ❌ Cause 6: Shaped velocity = 0 dù có position error lớn ⚠️
**Symptom:** Error RẤT LỚN nhưng velocity command = 0
```
[DEBUG] Kinematics: err=(-40.83,169.55,1.97) vel_cmd=(0.00,-0.00,0.00)
[DEBUG] FF: alignment=1.00 dist_scale=1.00 ff_scale=1.00
```

**Root Cause Analysis:**
- Position error = ~173m (RẤT LỚN!)
- Sqrt controller PHẢI tạo velocity khác 0
- Nhưng output = 0 → Kinematic shaping có vấn đề

**Debug với enhanced logs:**
```bash
./src/drone_follow_offboard/scripts/launch_debug.sh
# Xem:
# 1. Sqrt controller output: vel_target=(?,?)
# 2. After shaping: shaped_vel=(?,?)
# 3. Transform: yaw=? shaped_xy=(?,?) → body(fwd=?,left=?)
```

**Possible Issues:**
1. `shaped_vel_xy_` = (0,0) sau shaping → Shaping function có bug
2. Sqrt controller trả về (0,0) → k_v_xy quá lớn
3. Target acceleration input sai → Shaping reset về 0

**Solution:**
```bash
# Kiểm tra parameters
ros2 param get /smart_follow_node kp_pos
ros2 param get /smart_follow_node accel_max_ne
ros2 param get /smart_follow_node jerk_max_ne

# Tính k_v_xy = jerk_max / accel_max
# Nếu k_v_xy > 5 → quá lớn, response rất chậm
```

---

## Debug Output Examples

### ✅ Normal Following (có movement)
```
[DEBUG] Kinematics: drone_pos=(0.0,0.0,5.0) target_pos=(10.0,15.0,2.0) target_vel=(1.50,0.80)
[INFO] CMD: fwd=2.35 left=1.20 up=-0.50 yaw=0.15
```

### ⚠️ On Target (không cần di chuyển)
```
[DEBUG] Kinematics: drone_pos=(10.0,15.0,5.0) target_pos=(10.1,15.0,5.0) target_vel=(0.00,0.00)
[DEBUG] CMD: All zeros (on target or no movement needed)
```

### ❌ Target Not Valid
```
[DEBUG] Target not valid - stopping drone
```

---

## Force Movement Test

### Test với cố định NED offset
```bash
# Force drone follow 8m behind (South) + 2m right (East)
ros2 run drone_offboard smart_follow_node --ros-args \
    -p offset_type:=0 \
    -p offset_north:=-8.0 \
    -p offset_east:=2.0 \
    -p offset_down:=0.0 \
    -p follow_height:=5.0 \
    --log-level debug
```

Với NED offset, drone SẼ di chuyển ngay cả khi target đứng yên.

---

## Expected Behavior per Offset Mode

### VELOCITY Mode (offset_type=2) - DEFAULT
```
offset = -target_vel / |vel| * follow_dist
```
- ✅ Target đang di chuyển → Drone follow phía sau theo hướng di chuyển
- ❌ Target đứng yên → offset=0 → Drone ở ngay vị trí target

### NED Mode (offset_type=0)
```
offset = (offset_north, offset_east) cố định
```
- ✅ Luôn maintain offset cố định
- ✅ Hoạt động cả khi target đứng yên

### RELATIVE Mode (offset_type=1)
```
offset = rotate(offset_ned, target_heading)
```
- ✅ Offset xoay theo hướng target
- ✅ Hoạt động khi target đứng yên

---

## Quick Fix Commands

### Switch to NED mode with 8m behind offset
```bash
ros2 param set /smart_follow_node offset_type 0
ros2 param set /smart_follow_node offset_north -8.0
ros2 param set /smart_follow_node offset_east 0.0
```

### Increase follow distance
```bash
ros2 param set /smart_follow_node follow_dist 15.0
```

### Check current params
```bash
ros2 param list /smart_follow_node | grep offset
ros2 param get /smart_follow_node offset_type
```

---

## Root Cause Analysis Tree

```
CMD velocity = 0
    |
    ├─ Target not valid?
    │   └─ Check: [DEBUG] Target not valid
    │
    ├─ Target timeout?
    │   └─ Check: [WARN] Target timeout
    │
    ├─ Mode not GUIDED?
    │   └─ Check: Status Update -> Guided: NO
    │
    ├─ Target velocity = 0 AND offset_type = VELOCITY?
    │   └─ Solution: Switch to NED mode
    │
    ├─ Position error < threshold?
    │   └─ Already on target (normal)
    │
    └─ Sanity check failed?
        └─ Check: [ERROR] Velocity sanity check failed
```

---

**Run debug script để xem exact reason:**
```bash
./src/drone_follow_offboard/scripts/launch_debug.sh
```
