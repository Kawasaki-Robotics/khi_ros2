// Copyright 2025 Kawasaki Heavy Industries, Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KHI_HARDWARE__KHI_HARDWARE_INTERFACE_HPP_
#define KHI_HARDWARE__KHI_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_hardware/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace khi_hardware
{
class KhiPublisher;
class KhiService;

class KhiHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KhiHardwareInterface)

  KhiHardwareInterface() = default;
  ~KhiHardwareInterface() override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KHI_ROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* duration */) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* duration */) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  KHI_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  const KhiRobot & get_robot_info() { return driver_->get_robot(); }

private:
  int get_arm_no(const hardware_interface::ComponentInfo & joint) const;
  KhiRobotArmData get_arm_info(const int target_arm_no) const;
  void create_khi_robot_driver();

  bool is_cleaning_up_ = false;
  bool is_deactivating_ = false;
  bool is_shutdowning_ = false;
  std::shared_ptr<KhiDriver> driver_;
  std::shared_ptr<KhiPublisher> publisher_;
  std::shared_ptr<KhiService> service_;
};

}  // namespace khi_hardware

#endif  // KHI_HARDWARE__KHI_HARDWARE_INTERFACE_HPP_
