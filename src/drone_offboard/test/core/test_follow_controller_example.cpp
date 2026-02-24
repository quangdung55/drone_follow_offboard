/**
 * @file test_follow_controller.cpp
 * @brief Unit test example for FollowController (NO ROS NEEDED!)
 * 
 * This demonstrates the key benefit of refactoring:
 * - Pure C++ unit tests
 * - No ROS mocking needed
 * - Fast execution
 * - Easy to debug
 * 
 * Compile with:
 *   g++ -std=c++17 test_follow_controller.cpp \
 *       ../src/core/control/follow_controller.cpp \
 *       -I../include -lEigen3 -lgtest -lgtest_main
 */

#include <gtest/gtest.h>
#include "drone_offboard/core/control/follow_controller.hpp"
#include "drone_offboard/core/types/drone_state.hpp"
#include "drone_offboard/core/types/target_state.hpp"

using namespace drone_follow::core;

/**
 * @brief Test fixture for FollowController tests
 */
class FollowControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup default parameters
        params_.kp_pos = 0.1;
        params_.follow_height = 3.0;
        params_.follow_dist = 5.0;
        params_.accel_max_ne = 2.5;
        params_.jerk_max_ne = 5.0;
        params_.accel_max_d = 1.0;
        params_.jerk_max_d = 2.0;
        
        controller_ = std::make_unique<FollowController>(params_);
        
        // Setup default drone state
        drone_.pos_enu << 0.0, 0.0, 0.0;
        drone_.alt_rel = 0.0;
        drone_.yaw = 0.0;  // Facing East
        drone_.gps_valid = true;
        drone_.pose_valid = true;
        
        // Setup default target state
        target_.pos_enu << 10.0, 0.0, 0.0;  // 10m East
        target_.vel_enu.setZero();
        target_.accel_enu.setZero();
        target_.heading_rad = 0.0;
        target_.valid = true;
    }
    
    FollowControlParams params_;
    std::unique_ptr<FollowController> controller_;
    DroneState drone_;
    TargetState target_;
};

// =============================================================================
// BASIC TESTS
// =============================================================================

/**
 * @brief Test 1: Drone should move forward when target is ahead
 */
TEST_F(FollowControllerTest, MoveForwardWhenTargetAhead) {
    NavCommand cmd = controller_->update(drone_, target_, 0.02);
    
    EXPECT_GT(cmd.vel_forward, 0.0) << "Should command forward velocity";
    EXPECT_NEAR(cmd.vel_left, 0.0, 0.1) << "Should not move left/right";
}

/**
 * @brief Test 2: Drone should stop when at target position
 */
TEST_F(FollowControllerTest, StopWhenAtTarget) {
    drone_.pos_enu = target_.pos_enu;  // Same position
    
    NavCommand cmd = controller_->update(drone_, target_, 0.02);
    
    EXPECT_NEAR(cmd.vel_forward, 0.0, 0.1) << "Should not move when at target";
    EXPECT_NEAR(cmd.vel_left, 0.0, 0.1);
}

/**
 * @brief Test 3: Vertical control - should move up
 */
TEST_F(FollowControllerTest, MoveUpWhenBelowTarget) {
    drone_.alt_rel = 0.0;       // On ground
    params_.follow_height = 5.0; // Want 5m height
    controller_->set_params(params_);
    
    NavCommand cmd = controller_->update(drone_, target_, 0.02);
    
    EXPECT_GT(cmd.vel_up, 0.0) << "Should command upward velocity";
}

/**
 * @brief Test 4: Vertical control - should move down
 */
TEST_F(FollowControllerTest, MoveDownWhenAboveTarget) {
    drone_.alt_rel = 10.0;      // 10m high
    params_.follow_height = 3.0; // Want 3m height
    controller_->set_params(params_);
    
    NavCommand cmd = controller_->update(drone_, target_, 0.02);
    
    EXPECT_LT(cmd.vel_up, 0.0) << "Should command downward velocity";
}

// =============================================================================
// BODY FRAME TRANSFORMATION TESTS
// =============================================================================

/**
 * @brief Test 5: Body frame transformation - Facing North
 */
TEST_F(FollowControllerTest, BodyFrameTransformNorth) {
    drone_.pos_enu << 0.0, 0.0, 0.0;
    drone_.yaw = M_PI / 2.0;  // Facing North
    
    target_.pos_enu << 0.0, 10.0, 0.0;  // Target is North
    
    NavCommand cmd = controller_->update(drone_, target_, 0.02);
    
    EXPECT_GT(cmd.vel_forward, 0.0) << "Should move forward (North in body frame)";
}

/**
 * @brief Test 6: Body frame transformation - Facing South
 */
TEST_F(FollowControllerTest, BodyFrameTransformSouth) {
    drone_.pos_enu << 0.0, 0.0, 0.0;
    drone_.yaw = -M_PI / 2.0;  // Facing South
    
    target_.pos_enu << 0.0, -10.0, 0.0;  // Target is South
    
    NavCommand cmd = controller_->update(drone_, target_, 0.02);
    
    EXPECT_GT(cmd.vel_forward, 0.0) << "Should move forward (South in body frame)";
}

// =============================================================================
// FEEDFORWARD TESTS
// =============================================================================

/**
 * @brief Test 7: Target velocity feedforward
 */
TEST_F(FollowControllerTest, VelocityFeedforward) {
    target_.vel_enu << 5.0, 0.0, 0.0;  // Target moving East at 5 m/s
    
    // Multiple updates to let feedforward take effect
    NavCommand cmd;
    for (int i = 0; i < 10; i++) {
        cmd = controller_->update(drone_, target_, 0.02);
        drone_.pos_enu += Eigen::Vector3d(cmd.vel_forward * 0.02, 0, 0);
    }
    
    // Should have significant forward velocity due to feedforward
    EXPECT_GT(cmd.vel_forward, 2.0) << "Should have feedforward velocity";
}

// =============================================================================
// SAFETY TESTS
// =============================================================================

/**
 * @brief Test 8: Reset on large position jump
 */
TEST_F(FollowControllerTest, ResetOnLargeJump) {
    // First update
    NavCommand cmd1 = controller_->update(drone_, target_, 0.02);
    
    // Large position jump (GPS glitch)
    target_.pos_enu << 100.0, 0.0, 0.0;
    
    NavCommand cmd2 = controller_->update(drone_, target_, 0.02);
    
    // Controller should have reset internal state
    EXPECT_LT(cmd2.vel_forward, cmd1.vel_forward + 5.0) 
        << "Should not have huge velocity jump after position reset";
}

// =============================================================================
// KINEMATIC SHAPING TESTS
// =============================================================================

/**
 * @brief Test 9: Velocity smoothing (jerk limiting)
 */
TEST_F(FollowControllerTest, VelocitySmoothing) {
    std::vector<double> velocities;
    
    // Run multiple updates
    for (int i = 0; i < 50; i++) {
        NavCommand cmd = controller_->update(drone_, target_, 0.02);
        velocities.push_back(cmd.vel_forward);
        
        // Simulate drone movement
        drone_.pos_enu.x() += cmd.vel_forward * 0.02;
    }
    
    // Check that velocity changes smoothly (no sudden jumps)
    for (size_t i = 1; i < velocities.size(); i++) {
        double dv = std::abs(velocities[i] - velocities[i-1]);
        double max_dv = params_.jerk_max_ne * 0.02 * 0.02;  // dt^2 * jerk
        
        EXPECT_LT(dv, max_dv * 2.0) << "Velocity should change smoothly";
    }
}

/**
 * @brief Test 10: Acceleration limiting
 */
TEST_F(FollowControllerTest, AccelerationLimiting) {
    params_.accel_max_ne = 1.0;  // Low acceleration limit
    controller_->set_params(params_);
    
    target_.pos_enu << 100.0, 0.0, 0.0;  // Far away
    
    // Initial update
    NavCommand cmd1 = controller_->update(drone_, target_, 0.02);
    
    // Next update
    NavCommand cmd2 = controller_->update(drone_, target_, 0.02);
    
    // Acceleration should be limited
    double accel = (cmd2.vel_forward - cmd1.vel_forward) / 0.02;
    EXPECT_LT(accel, params_.accel_max_ne * 1.5) 
        << "Acceleration should be limited";
}

// =============================================================================
// MAIN
// =============================================================================
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/**
 * Expected output:
 * 
 * [==========] Running 10 tests from 1 test suite.
 * [----------] Global test environment set-up.
 * [----------] 10 tests from FollowControllerTest
 * [ RUN      ] FollowControllerTest.MoveForwardWhenTargetAhead
 * [       OK ] FollowControllerTest.MoveForwardWhenTargetAhead (0 ms)
 * [ RUN      ] FollowControllerTest.StopWhenAtTarget
 * [       OK ] FollowControllerTest.StopWhenAtTarget (0 ms)
 * ...
 * [----------] 10 tests from FollowControllerTest (5 ms total)
 * 
 * [==========] 10 tests from 1 test suite ran. (5 ms total)
 * [  PASSED  ] 10 tests.
 */
