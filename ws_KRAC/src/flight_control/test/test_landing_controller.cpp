#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "flight_control/precision_landing_controller.hpp"

using namespace flight_control;

TEST(PrecisionLandingControllerTest, PID_ProportionalResponse) {
    LandingControllerConfig cfg;
    cfg.kp = 1.0f;
    cfg.ki = 0.0f;
    cfg.kd = 0.0f;
    cfg.max_horizontal_vel = 1.5f;

    PrecisionLandingController controller(cfg);

    // Target offset in FRD: error = (0.5m forward, 0.2m right)
    // Current body velocity = (0, 0)
    auto cmd = controller.update(0.5f, 0.2f, 3.0f, 0.0f, 0.0f, 0.1f);

    EXPECT_NEAR(cmd.vx_frd, 0.5f, 1e-3);
    EXPECT_NEAR(cmd.vy_frd, 0.2f, 1e-3);
}

TEST(PrecisionLandingControllerTest, PID_VelocityDamping) {
    LandingControllerConfig cfg;
    cfg.kp = 1.0f;
    cfg.ki = 0.0f;
    cfg.kd = 0.5f;
    cfg.max_horizontal_vel = 1.5f;

    PrecisionLandingController controller(cfg);

    // Error = 0.5m, but drone is already moving forward at 0.4 m/s
    // Output should be Kp * error - Kd * curr_vel = 1.0 * 0.5 - 0.5 * 0.4 = 0.3 m/s
    auto cmd = controller.update(0.5f, 0.0f, 3.0f, 0.4f, 0.0f, 0.1f);

    EXPECT_NEAR(cmd.vx_frd, 0.3f, 1e-3);
}

TEST(PrecisionLandingControllerTest, PID_AntiWindup) {
    LandingControllerConfig cfg;
    cfg.kp = 0.0f;
    cfg.ki = 1.0f;
    cfg.kd = 0.0f;
    cfg.max_integral = 0.3f;

    PrecisionLandingController controller(cfg);

    // Simulate 50 steps of steady error = 0.5m
    LandingVelocityCommand cmd;
    for (int i = 0; i < 50; ++i) {
        cmd = controller.update(0.5f, 0.0f, 3.0f, 0.0f, 0.0f, 0.1f);
    }

    // Integral must clamp at max_integral = 0.3
    EXPECT_NEAR(cmd.vx_frd, 0.3f, 1e-3);
}

TEST(PrecisionLandingControllerTest, DescentFunnel_Gating) {
    LandingControllerConfig cfg;
    cfg.nominal_descent_vel = 0.5f;
    cfg.min_descent_vel = 0.05f;
    cfg.funnel_ratio = 0.2f; // funnel radius = max(0.2m, 0.2 * altitude)

    PrecisionLandingController controller(cfg);

    // At altitude 5.0m, funnel radius = 1.0m.
    // Case A: Centered (error = 0.05m) -> descent speed should be near nominal (0.5 m/s)
    auto cmd_centered = controller.update(0.05f, 0.0f, 5.0f, 0.0f, 0.0f, 0.1f);
    EXPECT_GT(cmd_centered.vz, 0.4f);

    // Case B: Far outside funnel (error = 2.5m) -> descent speed should be throttled to min_descent_vel
    auto cmd_offcenter = controller.update(2.5f, 0.0f, 5.0f, 0.0f, 0.0f, 0.1f);
    EXPECT_NEAR(cmd_offcenter.vz, 0.05f, 1e-3);
}

TEST(PrecisionLandingControllerTest, SmoothDeceleration_NearCenter) {
    LandingControllerConfig cfg;
    cfg.kp = 0.4f;
    cfg.ki = 0.0f;
    cfg.kd = 0.0f;

    PrecisionLandingController controller(cfg);

    // At small error = 0.05m, velocity must scale smoothly down to 0.02 m/s without discontinuous floor
    auto cmd = controller.update(0.05f, 0.0f, 0.4f, 0.0f, 0.0f, 0.1f);
    EXPECT_NEAR(cmd.vx_frd, 0.02f, 1e-4);
    EXPECT_NEAR(cmd.vy_frd, 0.0f, 1e-4);
}

TEST(PrecisionLandingControllerTest, TargetLoss_Hover) {
    LandingControllerConfig cfg;
    cfg.nominal_descent_vel = 0.5f;

    PrecisionLandingController controller(cfg);

    // Target error is NaN (lost target)
    float nan = std::numeric_limits<float>::quiet_NaN();
    auto cmd = controller.update(nan, nan, 3.0f, 0.0f, 0.0f, 0.1f);

    // In target loss, horizontal velocity and descent must be zero (hover)
    EXPECT_FLOAT_EQ(cmd.vx_frd, 0.0f);
    EXPECT_FLOAT_EQ(cmd.vy_frd, 0.0f);
    EXPECT_FLOAT_EQ(cmd.vz, 0.0f);
    EXPECT_TRUE(cmd.target_lost);
}

TEST(PrecisionLandingControllerTest, YawRate_HalfSpeedSmoothing) {
    LandingControllerConfig cfg;
    cfg.target_yaw = 0.0f;
    cfg.max_yaw_rate = 0.3927f; // 22.5 deg/s (half of default 45 deg/s)

    PrecisionLandingController controller(cfg);

    // Initial heading is 90 deg (1.5708 rad)
    float initial_yaw = 1.5708f;
    auto cmd0 = controller.update_yaw(initial_yaw, 0.1f);
    EXPECT_NEAR(cmd0.yaw, initial_yaw, 1e-3);

    // Simulate 10 steps (1.0 second) at dt = 0.1s
    PrecisionLandingController::YawCommand cmd;
    for (int i = 0; i < 10; ++i) {
        cmd = controller.update_yaw(initial_yaw, 0.1f);
    }

    // In 1 second, it should have turned by ~0.3927 rad (22.5 deg) towards 0
    float expected_yaw = initial_yaw - 0.3927f;
    EXPECT_NEAR(cmd.yaw, expected_yaw, 5e-3);
    EXPECT_NEAR(cmd.yawspeed, -0.3927f, 1e-3);
}
