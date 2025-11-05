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

#ifndef KHI_HARDWARE__KHI_ROBOT_HPP_
#define KHI_HARDWARE__KHI_ROBOT_HPP_

#include <string>
#include <vector>

namespace khi_hardware
{
struct KhiRobotArmData
{
  int joint_num{};
  std::string name;
  std::vector<std::string> joint_names{};
  std::vector<std::string> joint_types{};
  std::vector<double> min_positions{};
  std::vector<double> max_positions{};
  std::vector<double> command_positions{};
  std::vector<double> old_command_positions{};
  std::vector<double> state_positions{};
  std::vector<double> state_velocities{};
  std::vector<double> state_efforts{};
  std::vector<std::string> control_modes{};
};

struct KhiFTSensor
{
  double force_x;
  double force_y;
  double force_z;
  double torque_x;
  double torque_y;
  double torque_z;
  double counter_to_n_ratio;        // force [N] = counter_value * counter_to_n_ratio
  double counter_to_nm_ratio;       // torque [N] = counter_value * counter_to_nm_ratio
  bool enable_n_nm_output = false;  // true: output N or Nm, false: output counter value
};

struct KhiRobot
{
  std::string name;
  std::string ip_address;
  double period;
  int controller_no;
  std::vector<KhiRobotArmData> arms;
  KhiFTSensor ft_sensor;
};
}  // namespace khi_hardware
#endif  // KHI_HARDWARE__KHI_ROBOT_HPP_
