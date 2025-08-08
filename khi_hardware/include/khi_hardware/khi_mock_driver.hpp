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

#ifndef KHI_HARDWARE__KHI_MOCK_DRIVER_HPP_
#define KHI_HARDWARE__KHI_MOCK_DRIVER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_periodic_data_config.hpp"
#include "khi_hardware/khi_result_code.hpp"
#include "khi_msgs/srv/exec_khi_command.hpp"
#include "khi_msgs/srv/get_signal.hpp"
#include "khi_msgs/srv/reset_error.hpp"
#include "khi_msgs/srv/set_signal.hpp"
#include "khi_msgs/srv/set_timeout.hpp"

namespace khi_hardware
{
class KhiMockDriver : public KhiDriver
{
public:
  explicit KhiMockDriver(KhiRobot robot, KhiPeriodicDataConfig periodic_data)
  : KhiDriver(std::move(robot), periodic_data)
  {
  }
  ~KhiMockDriver() override = default;
  KhiResultCode initialize() override { return KhiResultCode::SUCCESS; }
  KhiResultCode configure() override { return KhiResultCode::SUCCESS; }
  KhiResultCode activate() override { return KhiResultCode::SUCCESS; }
  KhiResultCode deactivate() override { return KhiResultCode::SUCCESS; }
  KhiResultCode cleanup() override { return KhiResultCode::SUCCESS; }
  KhiResultCode shutdown() override { return KhiResultCode::SUCCESS; }
  KhiResultCode error() override { return KhiResultCode::SUCCESS; }
  bool read() override;
  bool write() override;
  bool is_writable() override { return true; };
  void get_signal_srv_cb(
    const khi_msgs::srv::GetSignal::Request::SharedPtr & /*req*/,
    const khi_msgs::srv::GetSignal::Response::SharedPtr & /*resp*/) const override
  {
  }
  void set_signal_srv_cb(
    const khi_msgs::srv::SetSignal::Request::SharedPtr & /*req*/,
    const khi_msgs::srv::SetSignal::Response::SharedPtr & /*resp*/) const override
  {
  }
  void exec_khi_command_srv_cb(
    const khi_msgs::srv::ExecKhiCommand::Request::SharedPtr & /*req*/,
    const khi_msgs::srv::ExecKhiCommand::Response::SharedPtr & /*resp*/) const override {};
  void reset_error_srv_cb(
    const khi_msgs::srv::ResetError::Request::SharedPtr & /*req*/,
    const khi_msgs::srv::ResetError::Response::SharedPtr & /*resp*/,
    const int /*arm_no*/) const override {};
  void set_timeout_srv_cb(
    const khi_msgs::srv::SetTimeout::Request::SharedPtr & /*req*/,
    const khi_msgs::srv::SetTimeout::Response::SharedPtr & /*resp*/) const override {};
  bool is_error() const override { return false; }
  bool get_error_info(
    std::vector<int> & /*error_codes*/, std::vector<std::string> & /*error_msgs*/) const override
  {
    return false;
  }
  bool get_actual_current(std::vector<float> & /*actual_current*/) const override { return false; }
  bool is_communicating() const override { return true; }
  void monitor_robot_health() override {}
};

}  // namespace khi_hardware

#endif  // KHI_HARDWARE__KHI_MOCK_DRIVER_HPP_
