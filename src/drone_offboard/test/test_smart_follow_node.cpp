#include <gtest/gtest.h>
#include "drone_offboard/smart_follow_node.hpp"
#include <cmath>

// ============================================================================
// Test Helper Functions
// ============================================================================

class SmartFollowNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS 2 if not already initialized
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        // Cleanup if needed
    }
};

// ============================================================================
// Test GPS to ENU Conversion
// ============================================================================

TEST_F(SmartFollowNodeTest, GPSToENUDelta_SamePosition) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double lat_ref = 21.0285;  // Hanoi coordinates
    double lon_ref = 105.8542;
    double d_north, d_east;
    
    // Test: Same position should give zero distance
    node->gps_to_enu_delta(lat_ref, lon_ref, lat_ref, lon_ref, d_north, d_east);
    
    EXPECT_NEAR(d_north, 0.0, 1e-6);
    EXPECT_NEAR(d_east, 0.0, 1e-6);
}

TEST_F(SmartFollowNodeTest, GPSToENUDelta_NorthDirection) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double lat_ref = 21.0285;
    double lon_ref = 105.8542;
    double lat_target = lat_ref + 0.0001;  // Move north ~11m
    double lon_target = lon_ref;
    double d_north, d_east;
    
    node->gps_to_enu_delta(lat_ref, lon_ref, lat_target, lon_target, d_north, d_east);
    
    // Should be approximately 11 meters north (0.0001 deg * 111km/deg)
    EXPECT_GT(d_north, 10.0);
    EXPECT_LT(d_north, 12.0);
    EXPECT_NEAR(d_east, 0.0, 0.5);
}

TEST_F(SmartFollowNodeTest, GPSToENUDelta_EastDirection) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double lat_ref = 21.0285;
    double lon_ref = 105.8542;
    double lat_target = lat_ref;
    double lon_target = lon_ref + 0.0001;  // Move east
    double d_north, d_east;
    
    node->gps_to_enu_delta(lat_ref, lon_ref, lat_target, lon_target, d_north, d_east);
    
    EXPECT_NEAR(d_north, 0.0, 0.5);
    EXPECT_GT(d_east, 9.0);  // Less than north due to latitude
    EXPECT_LT(d_east, 11.0);
}

// ============================================================================
// Test Velocity Shaping
// ============================================================================

TEST_F(SmartFollowNodeTest, ShapeVelocity_NoChange) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double desired_vel = 2.0;
    double current_vel = 2.0;
    double dt = 0.05;  // 50ms
    double accel_max = 2.5;
    
    double result = node->shape_velocity(desired_vel, current_vel, dt, accel_max);
    
    // Should remain same if already at desired velocity
    EXPECT_DOUBLE_EQ(result, 2.0);
}

TEST_F(SmartFollowNodeTest, ShapeVelocity_WithinAccelLimit) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double desired_vel = 1.0;
    double current_vel = 0.0;
    double dt = 0.05;  // 50ms
    double accel_max = 2.5;  // max delta = 2.5 * 0.05 = 0.125 m/s
    
    double result = node->shape_velocity(desired_vel, current_vel, dt, accel_max);
    
    // Should increase by max_delta (0.125)
    EXPECT_NEAR(result, 0.125, 1e-6);
}

TEST_F(SmartFollowNodeTest, ShapeVelocity_SmallChange) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double desired_vel = 1.05;
    double current_vel = 1.0;
    double dt = 0.05;
    double accel_max = 2.5;
    
    double result = node->shape_velocity(desired_vel, current_vel, dt, accel_max);
    
    // Small change (0.05) is within limit (0.125), should reach desired
    EXPECT_DOUBLE_EQ(result, 1.05);
}

TEST_F(SmartFollowNodeTest, ShapeVelocity_NegativeAcceleration) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double desired_vel = 0.0;
    double current_vel = 2.0;
    double dt = 0.05;
    double accel_max = 2.5;  // max delta = 0.125
    
    double result = node->shape_velocity(desired_vel, current_vel, dt, accel_max);
    
    // Should decrease by max_delta
    EXPECT_NEAR(result, 1.875, 1e-6);  // 2.0 - 0.125
}

TEST_F(SmartFollowNodeTest, ShapeVelocity_ZeroDt) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double desired_vel = 2.0;
    double current_vel = 1.0;
    double dt = 0.0;
    double accel_max = 2.5;
    
    double result = node->shape_velocity(desired_vel, current_vel, dt, accel_max);
    
    // With dt=0, max_delta=0, should stay at current
    EXPECT_DOUBLE_EQ(result, 1.0);
}

// ============================================================================
// Test Edge Cases
// ============================================================================

TEST_F(SmartFollowNodeTest, ShapeVelocity_LargeTimeStep) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double desired_vel = 5.0;
    double current_vel = 0.0;
    double dt = 10.0;  // Unrealistically large dt
    double accel_max = 2.5;  // max_delta = 25.0
    
    double result = node->shape_velocity(desired_vel, current_vel, dt, accel_max);
    
    // Even with large dt, should reach desired (clamped to desired)
    EXPECT_DOUBLE_EQ(result, 5.0);
}

TEST_F(SmartFollowNodeTest, ShapeVelocity_HighAccelLimit) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double desired_vel = 3.0;
    double current_vel = 1.0;
    double dt = 0.1;
    double accel_max = 100.0;  // Very high limit
    
    double result = node->shape_velocity(desired_vel, current_vel, dt, accel_max);
    
    // Should reach desired immediately
    EXPECT_DOUBLE_EQ(result, 3.0);
}

TEST_F(SmartFollowNodeTest, GPSToENUDelta_SouthWestDirection) {
    auto node = std::make_shared<SmartFollowNode>();
    
    double lat_ref = 21.0285;
    double lon_ref = 105.8542;
    double lat_target = lat_ref - 0.0001;  // South
    double lon_target = lon_ref - 0.0001;  // West
    double d_north, d_east;
    
    node->gps_to_enu_delta(lat_ref, lon_ref, lat_target, lon_target, d_north, d_east);
    
    // Both should be negative
    EXPECT_LT(d_north, 0.0);
    EXPECT_LT(d_east, 0.0);
    EXPECT_GT(d_north, -12.0);
    EXPECT_LT(d_north, -10.0);
}

// ============================================================================
// Test Constants
// ============================================================================

TEST(ConstantsTest, EarthRadiusValid) {
    EXPECT_DOUBLE_EQ(R_EARTH, 6378137.0);
}

TEST(ConstantsTest, DegToRadValid) {
    EXPECT_NEAR(DEG_TO_RAD, M_PI / 180.0, 1e-10);
    
    // Verify conversion
    double angle_deg = 90.0;
    double angle_rad = angle_deg * DEG_TO_RAD;
    EXPECT_NEAR(angle_rad, M_PI / 2.0, 1e-10);
}

// ============================================================================
// Test NavCommand Struct
// ============================================================================

TEST(NavCommandTest, Initialization) {
    NavCommand cmd = {1.0, 2.0, 3.0, 0.5};
    
    EXPECT_DOUBLE_EQ(cmd.vel_forward, 1.0);
    EXPECT_DOUBLE_EQ(cmd.vel_left, 2.0);
    EXPECT_DOUBLE_EQ(cmd.vel_up, 3.0);
    EXPECT_DOUBLE_EQ(cmd.yaw_rate, 0.5);
}

TEST(NavCommandTest, ZeroInitialization) {
    NavCommand cmd = {0, 0, 0, 0};
    
    EXPECT_DOUBLE_EQ(cmd.vel_forward, 0.0);
    EXPECT_DOUBLE_EQ(cmd.vel_left, 0.0);
    EXPECT_DOUBLE_EQ(cmd.vel_up, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw_rate, 0.0);
}

// ============================================================================
// Test TargetState Struct
// ============================================================================

TEST(TargetStateTest, DefaultInitialization) {
    TargetState target;
    
    EXPECT_DOUBLE_EQ(target.lat, 0.0);
    EXPECT_DOUBLE_EQ(target.lon, 0.0);
    EXPECT_DOUBLE_EQ(target.alt, 0.0);
    EXPECT_DOUBLE_EQ(target.heading_rad, 0.0);
    EXPECT_DOUBLE_EQ(target.yaw_rate, 0.0);
    EXPECT_FALSE(target.valid);
}

// ============================================================================
// Test MapPosition Struct (NEW - Map Frame Architecture)
// ============================================================================

TEST(MapPositionTest, DefaultInitialization) {
    MapPosition pos;
    
    EXPECT_DOUBLE_EQ(pos.pos_enu.x(), 0.0);
    EXPECT_DOUBLE_EQ(pos.pos_enu.y(), 0.0);
    EXPECT_DOUBLE_EQ(pos.pos_enu.z(), 0.0);
    EXPECT_DOUBLE_EQ(pos.vel_enu.x(), 0.0);
    EXPECT_DOUBLE_EQ(pos.vel_enu.y(), 0.0);
    EXPECT_DOUBLE_EQ(pos.vel_enu.z(), 0.0);
    EXPECT_DOUBLE_EQ(pos.accel_enu.x(), 0.0);
    EXPECT_DOUBLE_EQ(pos.accel_enu.y(), 0.0);
    EXPECT_DOUBLE_EQ(pos.accel_enu.z(), 0.0);
}

TEST(MapPositionTest, VelocityNorm) {
    MapPosition pos;
    pos.vel_enu.x() = 3.0;  // East
    pos.vel_enu.y() = 4.0;  // North
    
    double speed = pos.vel_enu.head<2>().norm();
    EXPECT_DOUBLE_EQ(speed, 5.0);  // 3-4-5 triangle
}

TEST(MapPositionTest, ErrorCalculation) {
    MapPosition drone;
    drone.pos_enu.x() = 10.0;  // East
    drone.pos_enu.y() = 20.0;  // North
    
    MapPosition target;
    target.pos_enu.x() = 15.0;  // East
    target.pos_enu.y() = 30.0;  // North
    
    // Error = target - drone (Map-frame pure!)
    Eigen::Vector3d error = target.pos_enu - drone.pos_enu;
    
    EXPECT_DOUBLE_EQ(error.x(), 5.0);   // 5m East
    EXPECT_DOUBLE_EQ(error.y(), 10.0);  // 10m North
}

// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
