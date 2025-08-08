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

#ifndef KHI_HARDWARE__KHI_DRIVER_HPP_
#define KHI_HARDWARE__KHI_DRIVER_HPP_

#include <math.h>
#include <string>
#include <utility>
#include <vector>

#include "khi_hardware/khi_periodic_data_config.hpp"
#include "khi_hardware/khi_result_code.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_msgs/msg/error_info.hpp"
#include "khi_msgs/srv/exec_khi_command.hpp"
#include "khi_msgs/srv/get_signal.hpp"
#include "khi_msgs/srv/reset_error.hpp"
#include "khi_msgs/srv/set_signal.hpp"
#include "khi_msgs/srv/set_timeout.hpp"

namespace khi_hardware
{
class KhiDriver
{
public:
  explicit KhiDriver(KhiRobot robot, KhiPeriodicDataConfig periodic_data)
  : robot_(std::move(robot)), periodic_data_config_(periodic_data)
  {
  }
  virtual ~KhiDriver() = 0;
  virtual KhiResultCode initialize() = 0;
  virtual KhiResultCode configure() = 0;
  virtual KhiResultCode activate() = 0;
  virtual KhiResultCode deactivate() = 0;
  virtual KhiResultCode cleanup() = 0;
  virtual KhiResultCode shutdown() = 0;
  virtual KhiResultCode error() = 0;
  virtual bool read() = 0;
  virtual bool write() = 0;
  virtual bool is_writable() = 0;
  virtual void monitor_robot_health() = 0;

  virtual void get_signal_srv_cb(
    const khi_msgs::srv::GetSignal::Request::SharedPtr & req,
    const khi_msgs::srv::GetSignal::Response::SharedPtr & resp) const = 0;
  virtual void set_signal_srv_cb(
    const khi_msgs::srv::SetSignal::Request::SharedPtr & req,
    const khi_msgs::srv::SetSignal::Response::SharedPtr & resp) const = 0;
  virtual void exec_khi_command_srv_cb(
    const khi_msgs::srv::ExecKhiCommand::Request::SharedPtr & req,
    const khi_msgs::srv::ExecKhiCommand::Response::SharedPtr & resp) const = 0;
  virtual void reset_error_srv_cb(
    const khi_msgs::srv::ResetError::Request::SharedPtr & req,
    const khi_msgs::srv::ResetError::Response::SharedPtr & resp, int arm_no) const = 0;
  virtual void set_timeout_srv_cb(
    const khi_msgs::srv::SetTimeout::Request::SharedPtr & req,
    const khi_msgs::srv::SetTimeout::Response::SharedPtr & resp) const = 0;
  virtual bool is_error() const = 0;
  virtual bool get_error_info(
    std::vector<int> & error_codes, std::vector<std::string> & error_msgs) const = 0;
  virtual bool get_actual_current(std::vector<float> & actual_current) const = 0;
  virtual bool is_communicating() const = 0;

  KhiRobot & get_robot() { return robot_; }

  const KhiRobot & get_robot() const { return robot_; }

protected:
  std::string convert_encoding(const char * input, const char * dst, const char * src) const;

  static constexpr int KHI_MAX_SIG_SIZE = 512;
  static constexpr int M2MM = 1000;
  static constexpr int ON = -1;
  static constexpr int OFF = 0;
  static constexpr int PRINT_THRESH = 1000;
  static constexpr int ERROR_RESET_TIME = 200;  // [ms]
  static constexpr double MSEC2SEC = 0.001;
  static constexpr double RAD2DEG = 180 / M_PI;
  static constexpr double MM2M = 0.001;
  const std::string jt_type_prismatic_ = "prismatic";

  KhiRobot robot_;
  KhiPeriodicDataConfig periodic_data_config_;
};

}  // namespace khi_hardware

#endif  // KHI_HARDWARE__KHI_DRIVER_HPP_
