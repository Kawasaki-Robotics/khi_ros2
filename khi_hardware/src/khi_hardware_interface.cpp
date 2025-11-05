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

#include "khi_hardware/khi_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "khi_hardware/khi_krnx_driver.hpp"
#include "khi_hardware/khi_mock_driver.hpp"
#include "khi_hardware/khi_periodic_data_config.hpp"
#include "khi_hardware/khi_publisher.hpp"
#include "khi_hardware/khi_result_code.hpp"
#include "khi_hardware/khi_service.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
KhiHardwareInterface::~KhiHardwareInterface() { driver_->error(); }

hardware_interface::CallbackReturn KhiHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_init");
  if (
    hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize
  info_ = hardware_info;
  is_cleaning_up_ = false;
  is_deactivating_ = false;
  is_shutdowning_ = false;

  // Check
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check if there are any inconsistencies in command_interfaces
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"), "Joint '%s' has %d command interface. size error.",
        joint.name.c_str(), (int)joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"), "Joint '%s' command interface. type error.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check if there are any inconsistencies in state_interfaces
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"), "Joint '%s' has %d state interface. size error.",
        joint.name.c_str(), (int)joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"),
        "The first state_interfaces of Joint '%s' should be position.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"),
        "The second state_interfaces of Joint '%s' should be position.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"),
        "The third state_interfaces of Joint '%s' should be effort.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  create_khi_robot_driver();

  auto result = driver_->initialize();
  if (result == KhiResultCode::FAILURE)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (result == KhiResultCode::ERROR)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "initialize success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_configure");
  auto result = driver_->configure();
  if (result == KhiResultCode::FAILURE)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (result == KhiResultCode::ERROR)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  service_ = std::make_shared<KhiService>();
  publisher_ = std::make_shared<KhiPublisher>();
  service_->start(*driver_);
  publisher_->start(*driver_);

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "configure success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KhiHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & arm : driver_->get_robot().arms)
  {
    for (int jt = 0; jt < arm.joint_num; jt++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm.joint_names[jt], hardware_interface::HW_IF_POSITION, &arm.state_positions[jt]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm.joint_names[jt], hardware_interface::HW_IF_VELOCITY, &arm.state_velocities[jt]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm.joint_names[jt], hardware_interface::HW_IF_EFFORT, &arm.state_efforts[jt]));
    }
  }

  for (const auto & sensor : info_.sensors)
  {
    state_interfaces.emplace_back(sensor.name, "force.x", &driver_->get_robot().ft_sensor.force_x);
    state_interfaces.emplace_back(sensor.name, "force.y", &driver_->get_robot().ft_sensor.force_y);
    state_interfaces.emplace_back(sensor.name, "force.z", &driver_->get_robot().ft_sensor.force_z);
    state_interfaces.emplace_back(
      sensor.name, "torque.x", &driver_->get_robot().ft_sensor.torque_x);
    state_interfaces.emplace_back(
      sensor.name, "torque.y", &driver_->get_robot().ft_sensor.torque_y);
    state_interfaces.emplace_back(
      sensor.name, "torque.z", &driver_->get_robot().ft_sensor.torque_z);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KhiHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & arm : driver_->get_robot().arms)
  {
    for (int jt = 0; jt < arm.joint_num; jt++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm.joint_names[jt], hardware_interface::HW_IF_POSITION, &arm.command_positions[jt]));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_active");

  auto result = driver_->activate();
  if (result == KhiResultCode::FAILURE)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (result == KhiResultCode::ERROR)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "Activation successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_deactivating_ = true;
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_deactive");
  auto result = driver_->deactivate();
  if (result == KhiResultCode::FAILURE)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (result == KhiResultCode::ERROR)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  is_deactivating_ = false;
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "Deactivation successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KhiHardwareInterface::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* duration */)
{
  if (is_cleaning_up_ || is_shutdowning_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!driver_->is_communicating())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "Communication with the robot controller has been lost.");
    return hardware_interface::return_type::ERROR;
  }

  driver_->monitor_robot_health();

  if (!driver_->read())
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "read err");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KhiHardwareInterface::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* duration */)
{
  auto deactivate = [&]()
  {
    driver_->deactivate();
    set_state(rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE));
    return hardware_interface::return_type::OK;
  };

  if (is_cleaning_up_ || is_deactivating_ || is_shutdowning_)
  {
    return hardware_interface::return_type::OK;
  }

  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return hardware_interface::return_type::OK;
  }

  if (!driver_->is_communicating())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "Communication with the robot controller has been lost.");
    return hardware_interface::return_type::ERROR;
  }

  if (!driver_->is_writable())
  {
    deactivate();
    RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "deactivate");
    return hardware_interface::return_type::OK;
  }

  if (!driver_->write())
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "write err");
    deactivate();
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_cleanup");

  is_cleaning_up_ = true;
  service_.reset();
  publisher_.reset();

  auto result = driver_->cleanup();
  is_cleaning_up_ = false;
  if (result == KhiResultCode::FAILURE)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (result == KhiResultCode::ERROR)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "Cleanup successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_shutdown");

  is_shutdowning_ = true;
  service_.reset();
  publisher_.reset();

  auto result = driver_->shutdown();
  is_shutdowning_ = false;
  if (result == KhiResultCode::FAILURE)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (result == KhiResultCode::ERROR)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_error");

  service_.reset();
  publisher_.reset();

  auto result = driver_->error();
  if (result == KhiResultCode::FAILURE)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (result == KhiResultCode::ERROR)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

int KhiHardwareInterface::get_arm_no(const hardware_interface::ComponentInfo & joint) const
{
  int arm_no = 0;
  if (joint.parameters.at("arm").find("arm2") != std::string::npos)
  {
    arm_no = 1;
  }
  return arm_no;
}

KhiRobotArmData KhiHardwareInterface::get_arm_info(const int target_arm_no) const
{
  KhiRobotArmData arm;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (get_arm_no(joint) != target_arm_no)
    {
      continue;
    }

    arm.joint_types.push_back(joint.parameters.at("type"));
    arm.joint_names.push_back(joint.name);
    for (const auto & command_interface : joint.command_interfaces)
    {
      if (command_interface.name == hardware_interface::HW_IF_POSITION)
      {
        arm.max_positions.push_back(std::stod(command_interface.max));
        arm.min_positions.push_back(std::stod(command_interface.min));
      }
      arm.control_modes.push_back(command_interface.name);
    }
  }

  arm.joint_num = static_cast<int>(arm.joint_names.size());
  arm.command_positions.resize(arm.joint_num, 0);
  arm.old_command_positions.resize(arm.joint_num, 0);
  arm.state_positions.resize(arm.joint_num, 0);
  arm.state_velocities.resize(arm.joint_num, 0);
  arm.state_efforts.resize(arm.joint_num, 0);

  return arm;
}

void KhiHardwareInterface::create_khi_robot_driver()
{
  // Get max arm number
  int max_arm_no = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    int arm_no = get_arm_no(joint);
    if (arm_no > max_arm_no)
    {
      max_arm_no = arm_no;
    }
  }

  // Make KhiRobotArmData
  std::vector<KhiRobotArmData> arms;
  for (int arm_no = 0; arm_no <= max_arm_no; arm_no++)
  {
    arms.push_back(get_arm_info(arm_no));
  }

  // Make KhiRobot
  KhiRobot robot;
  robot.controller_no = std::stoi(info_.hardware_parameters["controller_no"]);
  robot.name = info_.hardware_parameters["robot_name"];
  robot.ip_address = info_.hardware_parameters["robot_ip"];
  robot.period = 1000 / std::stoi(info_.hardware_parameters["update_rate"]);
  robot.arms = arms;

  // Make KhiPeriodicDataConfig
  KhiPeriodicDataConfig config = {};
  config.is_actual_current_enabled = (info_.hardware_parameters["actual_current"] == "True") ||
                                     (info_.hardware_parameters["actual_current"] == "true");
  config.is_actual_encorder_enabled = (info_.hardware_parameters["actual_encorder"] == "True") ||
                                      (info_.hardware_parameters["actual_encorder"] == "true");
  config.is_command_current_enabled = (info_.hardware_parameters["command_current"] == "True") ||
                                      (info_.hardware_parameters["command_current"] == "true");
  config.is_command_encorder_enabled = (info_.hardware_parameters["command_encorder"] == "True") ||
                                       (info_.hardware_parameters["command_encorder"] == "true");
  config.is_tcp_info_enabled = (info_.hardware_parameters["tcp_info"] == "True") ||
                               (info_.hardware_parameters["tcp_info"] == "true");
  config.is_external_signal_enabled = (info_.hardware_parameters["external_signal"] == "True") ||
                                      (info_.hardware_parameters["external_signal"] == "true");
  config.is_internal_signal_enabled = (info_.hardware_parameters["internal_signal"] == "True") ||
                                      (info_.hardware_parameters["internal_signal"] == "true");
  config.is_ft_sensor_enabled = (info_.hardware_parameters["ft_sensor"] == "True") ||
                                (info_.hardware_parameters["ft_sensor"] == "true");

  const auto is_simulation = (info_.hardware_parameters["simulation"] == "True") ||
                             (info_.hardware_parameters["simulation"] == "true");
  if (is_simulation)
  {
    driver_ = std::make_shared<KhiMockDriver>(robot, config);
    RCLCPP_INFO(
      rclcpp::get_logger("khi_hardware"), "KHI Robot Hardware Interface in simulation mode");
  }
  else
  {
    driver_ = std::make_shared<KhiKrnxDriver>(robot, config);
  }
}
}  // namespace khi_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(khi_hardware::KhiHardwareInterface, hardware_interface::SystemInterface)
