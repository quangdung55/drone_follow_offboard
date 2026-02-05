#include <gtest/gtest.h>
#include "drone_offboard/smart_follow_node.hpp"
#include <cmath>

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
    
    // Also check Map-ENU fields
    EXPECT_DOUBLE_EQ(target.pos_enu.x(), 0.0);
    EXPECT_DOUBLE_EQ(target.pos_enu.y(), 0.0);
    EXPECT_DOUBLE_EQ(target.pos_enu.z(), 0.0);
    EXPECT_DOUBLE_EQ(target.vel_enu.x(), 0.0);
    EXPECT_DOUBLE_EQ(target.vel_enu.y(), 0.0);
    EXPECT_DOUBLE_EQ(target.vel_enu.z(), 0.0);
}

TEST(TargetStateTest, VelocityNorm) {
    TargetState state;
    state.vel_enu.x() = 3.0;  // East
    state.vel_enu.y() = 4.0;  // North
    
    double speed = state.vel_enu.head<2>().norm();
    EXPECT_DOUBLE_EQ(speed, 5.0);  // 3-4-5 triangle
}

TEST(TargetStateTest, ErrorCalculation) {
    DroneState drone;
    drone.pos_enu.x() = 10.0;  // East
    drone.pos_enu.y() = 20.0;  // North
    
    TargetState target;
    target.pos_enu.x() = 15.0;  // East
    target.pos_enu.y() = 30.0;  // North
    
    // Error = target - drone (Map-frame pure!)
    Eigen::Vector3d error = target.pos_enu - drone.pos_enu;
    
    EXPECT_DOUBLE_EQ(error.x(), 5.0);   // 5m East
    EXPECT_DOUBLE_EQ(error.y(), 10.0);  // 10m North
}

// ============================================================================
// Test DroneState Struct
// ============================================================================

TEST(DroneStateTest, DefaultInitialization) {
    DroneState drone;
    
    EXPECT_DOUBLE_EQ(drone.lat, 0.0);
    EXPECT_DOUBLE_EQ(drone.lon, 0.0);
    EXPECT_DOUBLE_EQ(drone.alt_msl, 0.0);
    EXPECT_DOUBLE_EQ(drone.alt_rel, 0.0);
    EXPECT_DOUBLE_EQ(drone.yaw, 0.0);
    EXPECT_FALSE(drone.gps_valid);
    EXPECT_FALSE(drone.pose_valid);
    EXPECT_FALSE(drone.is_ready());
}

TEST(DroneStateTest, IsReady) {
    DroneState drone;
    
    // Not ready initially
    EXPECT_FALSE(drone.is_ready());
    
    // Only GPS valid
    drone.gps_valid = true;
    EXPECT_FALSE(drone.is_ready());
    
    // Only pose valid
    drone.gps_valid = false;
    drone.pose_valid = true;
    EXPECT_FALSE(drone.is_ready());
    
    // Both valid
    drone.gps_valid = true;
    drone.pose_valid = true;
    EXPECT_TRUE(drone.is_ready());
}

// ============================================================================
// Test AlphaBetaFilter
// ============================================================================

TEST(AlphaBetaFilterTest, DefaultInitialization) {
    AlphaBetaFilter filter;
    
    EXPECT_DOUBLE_EQ(filter.x, 0.0);
    EXPECT_DOUBLE_EQ(filter.v, 0.0);
}

TEST(AlphaBetaFilterTest, Reset) {
    AlphaBetaFilter filter;
    filter.x = 10.0;
    filter.v = 5.0;
    
    filter.reset(3.0);
    
    EXPECT_DOUBLE_EQ(filter.x, 3.0);
    EXPECT_DOUBLE_EQ(filter.v, 0.0);
}

TEST(AlphaBetaFilterTest, UpdateWithZeroDt) {
    AlphaBetaFilter filter;
    filter.reset(0.0);
    
    // Update with dt=0 should not change state
    filter.update(10.0, 0.0, 0.7, 0.1);
    
    EXPECT_DOUBLE_EQ(filter.x, 0.0);
    EXPECT_DOUBLE_EQ(filter.v, 0.0);
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
