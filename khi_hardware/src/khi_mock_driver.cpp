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

#include "khi_hardware/khi_mock_driver.hpp"
#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_msgs/srv/exec_khi_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{

bool KhiMockDriver::read()
{
  static KhiRobot prev_data = robot_;
  static int sim_cnt = 0;
  for (auto & arm : robot_.arms)
  {
    arm.state_positions = arm.command_positions;

    for (int jt = 0; jt < static_cast<int>(arm.joint_names.size()); jt++)
    {
      arm.state_velocities[jt] = arm.state_positions[jt] - arm.command_positions[jt];
    }
  }
  prev_data = robot_;
  if ((sim_cnt - 1) % PRINT_THRESH == 0)
  {
    constexpr int msg_size = 512;
    char msg[msg_size] = {0};
    int offset = 0;
    for (auto arm : robot_.arms)
    {
      for (int jt = 0; jt < arm.joint_num; jt++)
      {
        int written = snprintf(msg + offset, msg_size - offset, "%.3lf\t", arm.state_positions[jt]);
        if ((written < 0) || (written >= msg_size - offset))
        {
          break;
        }
        offset += written;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "[SIM] read %s", msg);
  }

  sim_cnt++;
  return true;
}

bool KhiMockDriver::write()
{
  static int sim_cnt = 0;
  if ((sim_cnt - 1) % PRINT_THRESH == 0)
  {
    constexpr int msg_size = 512;
    char msg[msg_size] = {0};
    int offset = 0;
    for (auto arm : robot_.arms)
    {
      for (int jt = 0; jt < arm.joint_num; jt++)
      {
        int written =
          snprintf(msg + offset, msg_size - offset, "%.3lf\t", arm.command_positions[jt]);
        if ((written < 0) || (written >= msg_size - offset))
        {
          break;
        }
        offset += written;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "[SIM] write %s", msg);
  }
  sim_cnt++;
  return true;
}
}  // namespace khi_hardware
