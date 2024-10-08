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

#ifndef diffbot_control__DIFFBOT_SYSTEM_HPP_
#define diffbot_control__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/node.hpp"

#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "diffbot_control/arduino_comms.hpp"
#include "diffbot_control/wheel.hpp"
#include "diffbot_control/imu.hpp"

namespace diffbot_control
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{


struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  std::string imu_name = "";
  float loop_rate = 0.0;
  std::string device_motor = "";
  std::string device_imu = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int imu_calib_timeout_ms = 0;
  int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
  

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr imu_calibrate_srv_;
  rclcpp::Node::SharedPtr node_;

  ArduinoComms comms_motor_;
  ArduinoComms comms_imu_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  Imu imu_;

  void imu_calibrate_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                              std::shared_ptr<std_srvs::srv::Empty::Response> response);
  
};

}  // namespace diffbot_control

#endif  // diffbot_control__DIFFBOT_SYSTEM_HPP_
