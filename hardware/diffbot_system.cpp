// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_2
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device_motor = info_.hardware_parameters["device_motor_bridge"];
  cfg_.device_imu = info_.hardware_parameters["device_imu_bridge"];

  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.imu_calib_timeout_ms = std::stoi(info_.hardware_parameters["imu_calib_timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.imu_name = info_.hardware_parameters["imu_name"];

  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"), "Configuring hardware with %s %s %f %s %s %d %d %d %d",
    cfg_.left_wheel_name.c_str(), cfg_.right_wheel_name.c_str(), cfg_.loop_rate, cfg_.device_motor.c_str(), cfg_.device_imu.c_str(),
    cfg_.baud_rate, cfg_.timeout_ms, cfg_.imu_calib_timeout_ms, cfg_.enc_counts_per_rev);

  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "PID values supplied: %d %d %d %d", cfg_.pid_p,
      cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
      
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  imu_.setup(cfg_.imu_name);


  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // hw_start_sec_ =
  //   hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  // hw_stop_sec_ =
  //   hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code
  // hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Add random ID to prevent warnings about multiple publishers within the same node
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name + "_sensor" });
  node_ = rclcpp::Node::make_shared("_", options);

  // Create publisher and service
  std::string topic_name = info_.name;
  std::transform(topic_name.begin(), topic_name.end(), topic_name.begin(), [](unsigned char c){ return std::tolower(c); });

  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_name + std::string("/imu"), 1);
  imu_calibrate_srv_ = node_->create_service<std_srvs::srv::Empty>(
    topic_name + std::string("/calibrate_imu"), std::bind(&DiffBotSystemHardware::imu_calibrate_callback, this, std::placeholders::_1, std::placeholders::_2));


  return hardware_interface::CallbackReturn::SUCCESS;
}

void DiffBotSystemHardware::imu_calibrate_callback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Calibrating IMU...please wait...");
   comms_imu_.calibrate_imu();
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  // IMU
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "orientation.x", &imu_.roll));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "orientation.y", &imu_.pitch));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "orientation.z", &imu_.yaw));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "angular_velocity.x", &imu_.gyro_x));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "angular_velocity.y", &imu_.gyro_y));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "angular_velocity.z", &imu_.gyro_z));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "linear_acceleration.x", &imu_.accel_x));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "linear_acceleration.y", &imu_.accel_y));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "linear_acceleration.z", &imu_.accel_z));
  


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Configuring ...please wait...");
  if (comms_motor_.connected())
  {
    comms_motor_.disconnect();
  }
  if (comms_imu_.connected())
  {
    comms_imu_.disconnect();
  }

  comms_motor_.connect(cfg_.device_motor, cfg_.baud_rate, cfg_.timeout_ms);
  comms_imu_.connect(cfg_.device_imu, cfg_.baud_rate, cfg_.timeout_ms, cfg_.imu_calib_timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured!");
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Calibrating IMU...please wait...");
   comms_imu_.calibrate_imu();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Cleaning up ...please wait...");
  if (comms_motor_.connected())
  {
    comms_motor_.disconnect();
  }
  if (comms_imu_.connected())
  {
    comms_imu_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");
  if (!comms_motor_.connected() || !comms_imu_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_motor_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Calibrating IMU...please wait...");
  // comms_imu_.calibrate_imu();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_motor_.connected() || !comms_imu_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_motor_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);
  comms_imu_.read_imu_values(imu_.accel_x, imu_.accel_y, imu_.accel_z, imu_.gyro_x, imu_.gyro_y, imu_.gyro_z);


  double delta_seconds = period.seconds();

  imu_.roll += imu_.gyro_x * delta_seconds;
  imu_.pitch += imu_.gyro_y * delta_seconds;
  imu_.yaw += imu_.gyro_z * delta_seconds;
  
  // std::cout<< "Roll: %f" << imu_.roll;
  // std::cout<< "Pitch: %f" << imu_.pitch;
  // std::cout<< "Yaw: %f" << imu_.yaw;

  // RCLCPP_INFO( rclcpp::get_logger("DiffBotSystemHardware"), "%f %f %f %f %f %f %f %f %f", imu_.accel_x, imu_.accel_y, imu_.accel_z, imu_.gyro_x, imu_.gyro_y, imu_.gyro_z, imu_.roll, imu_.pitch, imu_.yaw);


  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  imu_pub_->publish(imu_.get_imu_msg());



  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_2::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_motor_.connected() || !comms_imu_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_motor_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
