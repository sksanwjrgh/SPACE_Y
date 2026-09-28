#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace flight_control {

struct LandingControllerConfig {
    float kp = 0.4f;
    float ki = 0.01f;
    float kd = 0.0f;
    float max_horizontal_vel = 1.0f;
    float max_integral = 0.2f;
    float nominal_descent_vel = 0.5f;
    float min_descent_vel = 0.05f;
    float funnel_ratio = 0.25f;
    float min_funnel_radius = 0.20f;
    float target_yaw = 0.0f;        // desired heading in radians (0.0 = North)
    float max_yaw_rate = 0.3927f;   // 22.5 deg/s (half of default 45 deg/s auto yaw rate)
};

struct LandingVelocityCommand {
    float vx_frd = 0.0f; // forward velocity setpoint (m/s)
    float vy_frd = 0.0f; // right velocity setpoint (m/s)
    float vz = 0.0f;     // downward descent velocity setpoint (m/s)
    bool target_lost = false;
};

class PrecisionLandingController {
public:
    struct YawCommand {
        float yaw = 0.0f;
        float yawspeed = 0.0f;
    };

    explicit PrecisionLandingController(const LandingControllerConfig &config = LandingControllerConfig())
        : cfg_(config) {}

    void reset() {
        integral_x_ = 0.0f;
        integral_y_ = 0.0f;
        yaw_initialized_ = false;
    }

    YawCommand update_yaw(float current_yaw, float dt) {
        if (!yaw_initialized_) {
            current_yaw_setpoint_ = current_yaw;
            yaw_initialized_ = true;
            return {current_yaw_setpoint_, 0.0f};
        }

        if (dt <= 0.0f) dt = 0.1f;

        float diff = cfg_.target_yaw - current_yaw_setpoint_;
        while (diff > static_cast<float>(M_PI)) diff -= 2.0f * static_cast<float>(M_PI);
        while (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);

        float max_step = cfg_.max_yaw_rate * dt;
        float step = std::clamp(diff, -max_step, max_step);
        current_yaw_setpoint_ += step;

        while (current_yaw_setpoint_ > static_cast<float>(M_PI)) current_yaw_setpoint_ -= 2.0f * static_cast<float>(M_PI);
        while (current_yaw_setpoint_ < -static_cast<float>(M_PI)) current_yaw_setpoint_ += 2.0f * static_cast<float>(M_PI);

        float yawspeed = step / dt;
        return {current_yaw_setpoint_, yawspeed};
    }

    float current_yaw_setpoint() const { return current_yaw_setpoint_; }

    LandingVelocityCommand update(float target_x_frd, float target_y_frd, float altitude,
                                  float current_vx_frd, float current_vy_frd, float dt) {
        LandingVelocityCommand cmd;

        // Target Loss check: If coordinates are NaN or altitude is invalid
        if (!std::isfinite(target_x_frd) || !std::isfinite(target_y_frd) || !std::isfinite(altitude) || altitude <= 0.05f) {
            cmd.vx_frd = 0.0f;
            cmd.vy_frd = 0.0f;
            cmd.vz = 0.0f;
            cmd.target_lost = true;
            reset();
            return cmd;
        }

        cmd.target_lost = false;
        if (dt <= 0.0f) dt = 0.1f;

        float error_dist = std::hypot(target_x_frd, target_y_frd);

        // Anti-windup: Only integrate if error is within 1.0m
        if (error_dist < 1.0f) {
            integral_x_ += target_x_frd * dt;
            integral_y_ += target_y_frd * dt;

            integral_x_ = std::clamp(integral_x_, -cfg_.max_integral, cfg_.max_integral);
            integral_y_ = std::clamp(integral_y_, -cfg_.max_integral, cfg_.max_integral);
        } else {
            reset();
        }

        // PID calculation in FRD frame (smooth asymptotic convergence):
        float vx = cfg_.kp * target_x_frd + cfg_.ki * integral_x_ - cfg_.kd * current_vx_frd;
        float vy = cfg_.kp * target_y_frd + cfg_.ki * integral_y_ - cfg_.kd * current_vy_frd;

        float cmd_speed = std::hypot(vx, vy);

        // Velocity saturation clamp
        if (cmd_speed > cfg_.max_horizontal_vel) {
            float scale = cfg_.max_horizontal_vel / cmd_speed;
            vx *= scale;
            vy *= scale;
        }

        cmd.vx_frd = vx;
        cmd.vy_frd = vy;

        // Descent Funnel
        float funnel_radius = std::max(cfg_.min_funnel_radius, cfg_.funnel_ratio * altitude);
        if (error_dist <= funnel_radius) {
            float ratio = 1.0f - (error_dist / funnel_radius);
            cmd.vz = cfg_.min_descent_vel + ratio * (cfg_.nominal_descent_vel - cfg_.min_descent_vel);
        } else {
            cmd.vz = cfg_.min_descent_vel;
        }

        return cmd;
    }

private:
    LandingControllerConfig cfg_;
    float integral_x_ = 0.0f;
    float integral_y_ = 0.0f;
    float current_yaw_setpoint_ = 0.0f;
    bool yaw_initialized_ = false;
};

} // namespace flight_control
