#include <iostream>
#include <chrono>
#include <limits>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "flight_control/precision_landing_controller.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class LandingTest : public rclcpp::Node {
  public:
    LandingTest() : Node("landing") {
      odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
             curr_odom_ = *msg;
             has_odom_ = true;
      });

      landed_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
             landed_ = msg->landed;
      });

      desired_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/landing/coordinates", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
             desired_x_ = msg->point.x;
             desired_y_ = msg->point.y;
             altitude_ = msg->point.z;
             have_alt_  = std::isfinite(altitude_) && altitude_ > 0.05f; 
      });

      this->declare_parameter<float>("kp", 0.4f);
      this->declare_parameter<float>("ki", 0.01f);
      this->declare_parameter<float>("kd", 0.0f);
      this->declare_parameter<float>("max_horizontal_vel", 1.0f);
      this->declare_parameter<float>("nominal_descent_vel", 0.5f);
      this->declare_parameter<float>("min_descent_vel", 0.05f);
      this->declare_parameter<float>("low_enough", 0.4f);
      this->declare_parameter<float>("target_yaw", 0.0f);
      this->declare_parameter<float>("max_yaw_rate", 0.3927f);

      flight_control::LandingControllerConfig cfg;
      cfg.kp = (float)this->get_parameter("kp").as_double();
      cfg.ki = (float)this->get_parameter("ki").as_double();
      cfg.kd = (float)this->get_parameter("kd").as_double();
      cfg.max_horizontal_vel = (float)this->get_parameter("max_horizontal_vel").as_double();
      cfg.nominal_descent_vel = (float)this->get_parameter("nominal_descent_vel").as_double();
      cfg.min_descent_vel = (float)this->get_parameter("min_descent_vel").as_double();
      cfg.target_yaw = (float)this->get_parameter("target_yaw").as_double();
      cfg.max_yaw_rate = (float)this->get_parameter("max_yaw_rate").as_double();
      low_enough_ = (float)this->get_parameter("low_enough").as_double();
      controller_ = std::make_unique<flight_control::PrecisionLandingController>(cfg);

      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

      auto timer_callback = [this]() -> void {
        if(!has_odom_) {
          RCLCPP_WARN(this->get_logger(), "Waiting for...");
          return;
        }

        if(!armed_ && mission_mode_ != FINISHED) {
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          this->arm();
        }

        publish_offboard_control_mode();

        switch (mission_mode_) {
          default:

          case LANDING:
            land();
            break;

          case FINISHED:
            if(landed_ && armed_) disarm();
            {
              TrajectorySetpoint land_msg {};
              float nan = std::numeric_limits<float>::quiet_NaN();
              land_msg.position = {nan, nan, nan};
              land_msg.velocity = {0.0f, 0.0f, 0.3f};
              land_msg.yaw = controller_->current_yaw_setpoint();
              land_msg.yawspeed = 0.0f;
              land_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
              trajectory_setpoint_publisher_->publish(land_msg);
            }
            if(!armed_) return;
            break;
        }
        offboard_setpoint_counter_ ++;
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_sub_;

    px4_msgs::msg::VehicleOdometry curr_odom_;

    enum Mission {
      LANDING,
      FINISHED,
    };

    bool has_odom_ = false;
    bool armed_ = false;
    bool landed_ = false;
    bool have_alt_ = false;

    int hold_counter_ = 0;
    int HOLD_THRESHOLD = 20;

    void arm();
    void disarm();

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void land();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void publish_vehicle_command(int command, float value);

    int offboard_setpoint_counter_ = 0;

    float desired_x_ = 0.0f;
    float desired_y_ = 0.0f;
    float altitude_ = 0.0f;

    float low_enough_ = 0.4f;
    std::unique_ptr<flight_control::PrecisionLandingController> controller_;

    Mission mission_mode_ = LANDING;
};

void LandingTest::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
  armed_ = true;
}

void LandingTest::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
  armed_ = false;
}

void LandingTest::publish_offboard_control_mode() {
  OffboardControlMode msg {};
  msg.position = mission_mode_ == LANDING? false:true;
  msg.velocity = mission_mode_ == LANDING? true:false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void LandingTest::publish_trajectory_setpoint() {
  if(curr_odom_.timestamp == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  TrajectorySetpoint msg {};

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  Eigen::Vector3f target(5, -3, -10);
  Eigen::Vector3f to_wp = target - current;
  float dist_to_wp = to_wp.norm();

  RCLCPP_INFO(this->get_logger(), "[Multirotor] Distance to waypoint: %f", dist_to_wp);

  if(dist_to_wp < 1.0f) {
    hold_counter_ ++;

    if(hold_counter_ > HOLD_THRESHOLD) {
      hold_counter_ = 0;
      mission_mode_ = LANDING;
    }
  }

  msg.position = {target[0], target[1], target[2]};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::land() {
  if (!has_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for odometry...");
    return;
  }
  if (!have_alt_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /landing/coordinates (alt)...");
    return;
  }

  Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]);
  q.normalize();

  // Extract current yaw from quaternion
  float siny_cosp = 2.0f * (q.w() * q.z() + q.x() * q.y());
  float cosy_cosp = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
  float curr_yaw = std::atan2(siny_cosp, cosy_cosp);

  auto yaw_cmd = controller_->update_yaw(curr_yaw, 0.1f);

  // Drone velocity in NED -> body FRD for velocity damping
  Eigen::Vector3f vel_NED(curr_odom_.velocity[0], curr_odom_.velocity[1], curr_odom_.velocity[2]);
  Eigen::Vector3f vel_FRD = q.conjugate() * vel_NED;

  auto cmd = controller_->update(desired_x_, desired_y_, altitude_, vel_FRD.x(), vel_FRD.y(), 0.1f);

  Eigen::Vector3f cmd_vel_FRD(cmd.vx_frd, cmd.vy_frd, 0.0f);
  Eigen::Vector3f cmd_vel_NED = q * cmd_vel_FRD;

  if (altitude_ <= low_enough_) {
    mission_mode_ = FINISHED;
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "[Landing] Low enough at altitude %.3fm. Sending land command.", altitude_);
  }

  TrajectorySetpoint msg {};
  float nan = std::numeric_limits<float>::quiet_NaN();
  msg.position = {nan, nan, nan};
  msg.velocity = {cmd_vel_NED[0], cmd_vel_NED[1], cmd.vz};
  msg.yaw = yaw_cmd.yaw;
  msg.yawspeed = yaw_cmd.yawspeed;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::publish_vehicle_command(uint16_t command, float param1, float param2) {
  VehicleCommand msg {};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  std::cout << "Starting landing test" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingTest>());

  rclcpp::shutdown();
  return 0;
}
