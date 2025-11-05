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

#ifndef KHI_HARDWARE__KHI_KRNX_DRIVER_HPP_
#define KHI_HARDWARE__KHI_KRNX_DRIVER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_periodic_data_config.hpp"
#include "khi_hardware/khi_result_code.hpp"
#include "khi_hardware/krnx.h"
#include "khi_msgs/msg/error_info.hpp"
#include "khi_msgs/srv/change_ft_output_mode.hpp"
#include "khi_msgs/srv/exec_khi_command.hpp"
#include "khi_msgs/srv/get_signal.hpp"
#include "khi_msgs/srv/reset_error.hpp"
#include "khi_msgs/srv/set_ati_software_bias.hpp"
#include "khi_msgs/srv/set_signal.hpp"
#include "khi_msgs/srv/set_timeout.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
struct KhiRobot;

struct KrnxArmData
{
  float comp[KRNX_MAXAXES]{};
  int status[KRNX_MAXAXES]{};
  std::vector<double> home_positions{};
};

class KhiKrnxDriver : public KhiDriver
{
public:
  explicit KhiKrnxDriver(KhiRobot robot, KhiPeriodicDataConfig config)
  : KhiDriver(std::move(robot), config)
  {
  }
  ~KhiKrnxDriver() override = default;
  KhiResultCode initialize() override;
  KhiResultCode configure() override;
  KhiResultCode activate() override;
  KhiResultCode deactivate() override;
  KhiResultCode cleanup() override;
  KhiResultCode shutdown() override;
  KhiResultCode error() override;
  bool read() override;
  bool write() override;
  bool is_writable() override;
  void get_signal_srv_cb(
    const khi_msgs::srv::GetSignal::Request::SharedPtr & req,
    const khi_msgs::srv::GetSignal::Response::SharedPtr & resp) const override;
  void set_signal_srv_cb(
    const khi_msgs::srv::SetSignal::Request::SharedPtr & req,
    const khi_msgs::srv::SetSignal::Response::SharedPtr & resp) const override;
  void exec_khi_command_srv_cb(
    const khi_msgs::srv::ExecKhiCommand::Request::SharedPtr & req,
    const khi_msgs::srv::ExecKhiCommand::Response::SharedPtr & resp) const override;
  void reset_error_srv_cb(
    const khi_msgs::srv::ResetError::Request::SharedPtr & req,
    const khi_msgs::srv::ResetError::Response::SharedPtr & resp, int arm_no) const override;
  void set_timeout_srv_cb(
    const khi_msgs::srv::SetTimeout::Request::SharedPtr & req,
    const khi_msgs::srv::SetTimeout::Response::SharedPtr & resp) const override;
  void set_ati_software_bias_srv_cb(
    const khi_msgs::srv::SetATISoftwareBias::Request::SharedPtr & req,
    const khi_msgs::srv::SetATISoftwareBias::Response::SharedPtr & resp) const override;
  void change_ft_output_mode_srv_cb(
    const khi_msgs::srv::ChangeFTOutputMode::Request::SharedPtr & req,
    const khi_msgs::srv::ChangeFTOutputMode::Response::SharedPtr & resp) override;
  bool is_error() const override;
  bool get_error_info(
    std::vector<int> & error_codes, std::vector<std::string> & error_msgs) const override;
  bool get_actual_current(std::vector<float> & actual_current) const override;
  bool is_communicating() const override;
  void monitor_robot_health() override;

private:
  bool reset_home_position();
  bool chk_language();
  bool is_position_command_constant();

  bool has_met_ros_requirements() const;
  bool load_rtc_program() const;
  bool exec_rtc_program() const;
  bool is_program_running() const;
  bool is_configuration_valid() const;
  bool is_robot_software_version_valid() const;
  void report_write_error() const;
  bool power_on_motor() const;
  bool power_off_motor(bool need_log) const;
  bool get_motor_lamp(int & motor_lamp, const int arm_no, const bool need_log) const;
  bool hold(bool need_log) const;
  bool kill_program(bool need_log) const;
  bool reset_error() const;
  bool open() const;
  bool close(bool need_log) const;
  bool get_curmotion_data_ex(int cont_no, int robot_no, TKrnxCurMotionDataEx * p_motion_data) const;
  bool exec_monitor_command(
    int cont_no, const char * cmd, char * buffer, int buffer_sz, int * as_err_code,
    bool need_log) const;
  void convert_error_code(std::string & str_error_code, int error_code) const;
  std::string convert_to_utf8(const char * input) const;
  void handle_krnx_error(
    const std::string & krnx_api, const int krnx_return_code, const int as_err_code,
    const int arm_no) const;
  void set_periodic_data_config() const;

  static constexpr int KRNX_MSGSIZE = 1024;
  static constexpr int KRNX_CMD_SIZE = 256;
  static constexpr int KRNX_STDAXES = 6;
  static constexpr int KRNX_MOTION_BUF = 10;
  static constexpr int KRNX_BUFFER_SIZE = 4;
  static constexpr int KRNX_BUFFER_SIZE_THRESH = 20;
  static constexpr int KRNX_BUFFER_WARNING_INTERVAL = 1000;
  static constexpr int KRNX_AUX_TIMEOUT = 3000;  // [ms]
  static constexpr int KRNX_POS_UPPER_LIMIT_ERR = 0x01;
  static constexpr int KRNX_POS_LOWER_LIMIT_ERR = 0x02;
  static constexpr int KRNX_SPD_LIMIT_ERR = 0x04;

  enum KrnxFTSensor
  {
    FORCE_X = 3,
    FORCE_Y = 4,
    FORCE_Z = 5,
    TORQUE_X = 6,
    TORQUE_Y = 7,
    TORQUE_Z = 8,
  };

  int seq_no_ = 0;
  bool is_japanese_;
  bool is_chinese_;
  bool is_korean_;
  int cmd_constant_cnt_ = 0;
  int rtc_buffer_thresh_exceed_cnt_ = KRNX_BUFFER_WARNING_INTERVAL;
  std::vector<KrnxArmData> krnx_arm_data_{};
};

}  // namespace khi_hardware

#endif  // KHI_HARDWARE__KHI_KRNX_DRIVER_HPP_
