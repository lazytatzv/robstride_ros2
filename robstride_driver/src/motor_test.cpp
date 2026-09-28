#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robstride_driver/config.hpp"
#include "robstride_driver/driver.hpp"
#include "robstride_driver/motor_profile.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("robstride_motor_test");

  robstride_driver::DriverConfiguration config;

  config.settings.host_id = 0xfd;
  config.settings.transport.node_name = "robstride_motor_test";
  config.settings.transport.transmit_topic = "/robstride/can_tx";
  config.settings.transport.receive_topic = "/robstride/can_rx";
  config.settings.transport.motor_count = 1;

  robstride_driver::JointData motor;

  motor.name = "motor1";
  motor.can_id = 1;

  // EL05 protocol limits
  motor.limits = robstride_driver::motor_profile("EL05");

  // Driver requires a non-zero watchdog value.
  motor.can_timeout_ticks = 100;

  motor.kp = 20.0;
  motor.kd = 0.5;

  // Position command is the interface we are testing.
  motor.claimed.position = true;
  motor.claimed.velocity = false;
  motor.claimed.effort = false;

  // Use the motor profile limits as command limits.
  motor.command_limits.position_min = motor.limits.position_min;
  motor.command_limits.position_max = motor.limits.position_max;
  motor.command_limits.velocity_min = motor.limits.velocity_min;
  motor.command_limits.velocity_max = motor.limits.velocity_max;
  motor.command_limits.effort_min = motor.limits.effort_min;
  motor.command_limits.effort_max = motor.limits.effort_max;

  motor.command.position = 0.0;
  motor.command.velocity = 0.0;
  motor.command.effort = 0.0;

  config.joints.push_back(motor);

  robstride_driver::RobStrideDriver driver(node->get_logger());

  if (!driver.initialize(config)) {
    RCLCPP_ERROR(node->get_logger(), "initialize() failed");
    rclcpp::shutdown();
    return 1;
  }

  if (!driver.open()) {
    RCLCPP_ERROR(node->get_logger(), "open() failed");
    rclcpp::shutdown();
    return 1;
  }

  if (!driver.start()) {
    RCLCPP_ERROR(node->get_logger(), "start() failed");
    driver.close();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Motor started");

  auto & joint = driver.joints()[0];

  // Hold zero for 2 seconds.
  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok() &&
         std::chrono::steady_clock::now() - start_time < 2s)
  {
    joint.command.position = 0.0;

    driver.update_state();
    driver.send_commands();

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  // Move to +0.5 rad.
  start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok() &&
         std::chrono::steady_clock::now() - start_time < 3s)
  {
    joint.command.position = 0.5;

    driver.update_state();
    driver.send_commands();

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  // Return to zero.
  start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok() &&
         std::chrono::steady_clock::now() - start_time < 3s)
  {
    joint.command.position = 0.0;

    driver.update_state();
    driver.send_commands();

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  RCLCPP_INFO(node->get_logger(), "Stopping motor");

  driver.stop();
  driver.close();

  rclcpp::shutdown();
  return 0;
}