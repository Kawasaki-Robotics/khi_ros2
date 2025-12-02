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

#include <climits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_krnx_driver.hpp"
#include "khi_hardware/khi_result_code.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_msgs/msg/error_info.hpp"
#include "khi_msgs/srv/exec_khi_command.hpp"
#include "khi_msgs/srv/get_signal.hpp"
#include "khi_msgs/srv/set_signal.hpp"
#include "khi_msgs/srv/set_timeout.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
KhiResultCode KhiKrnxDriver::initialize()
{
  // Check KRNX Version
  char msg[KRNX_MSGSIZE] = {0};
  const int return_code = krnx_GetKrnxVersion(msg, sizeof(msg));
  if (return_code != KRNX_NOERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "krnx_GetKrnxVersion returned -0x%X", -return_code);
    return KhiResultCode::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "%s", msg);

  // Initialize
  seq_no_ = 0;
  krnx_arm_data_.resize(0);
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    krnx_arm_data_.push_back(KrnxArmData());
    krnx_arm_data_[arm_no].home_positions.resize(robot_.arms[arm_no].joint_num, 0);
    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      krnx_arm_data_[arm_no].comp[jt] = 0;
    }
  }
  is_japanese_ = false;
  is_chinese_ = false;
  is_korean_ = false;
  cmd_constant_cnt_ = 0;
  rtc_buffer_thresh_exceed_cnt_ = KRNX_BUFFER_WARNING_INTERVAL;

  return KhiResultCode::SUCCESS;
}

KhiResultCode KhiKrnxDriver::configure()
{
  set_periodic_data_config();

  if (!open())
  {
    return KhiResultCode::ERROR;
  }

  if (!is_configuration_valid())
  {
    return KhiResultCode::ERROR;
  }

  if (!is_robot_software_version_valid())
  {
    return KhiResultCode::ERROR;
  }

  if (is_program_running())
  {
    if (!hold(true))
    {
      return KhiResultCode::ERROR;
    }
  }

  // Set RtcInfo
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    TKrnxRtcInfo rtcont_info;
    rtcont_info.cyc = static_cast<int16_t>(robot_.period);
    rtcont_info.buf = KRNX_BUFFER_SIZE;
    rtcont_info.interpolation = 1;
    const int return_code = krnx_SetRtcInfo(robot_.controller_no, &rtcont_info);
    if (return_code != KRNX_NOERROR)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "krnx_SetRtcInfo returned -0x%X", -return_code);
      return KhiResultCode::ERROR;
    }
  }

  if (!load_rtc_program())
  {
    return KhiResultCode::ERROR;
  }

  if (!chk_language())
  {
    return KhiResultCode::FAILURE;
  }

  krnx_SetAuxApiTimeoutPeriod(robot_.controller_no, KRNX_AUX_TIMEOUT);

  int error_code = 0;
  char msg_buf[KRNX_MSGSIZE];
  exec_monitor_command(
    robot_.controller_no, "ZPATHCONST_CALTIMEREDUCE ON", msg_buf, sizeof(msg_buf), &error_code,
    true);

  return KhiResultCode::SUCCESS;
}

/**
 * @brief activate
 * @param cont_no Controller No
 */
KhiResultCode KhiKrnxDriver::activate()
{
  if (!is_communicating())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "Communication with the robot controller has been lost.");
    return KhiResultCode::ERROR;
  }

  if (!has_met_ros_requirements())
  {
    return KhiResultCode::FAILURE;
  }

  if (!reset_error())
  {
    return KhiResultCode::FAILURE;
  }

  if (!power_on_motor())
  {
    return KhiResultCode::FAILURE;
  }

  if (is_program_running())
  {
    if (!hold(true))
    {
      return KhiResultCode::FAILURE;
    }
  }

  // Clear RTC Comp Data
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    const int return_code = krnx_OldCompClear(robot_.controller_no, arm_no);
    if (return_code != KRNX_NOERROR)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "krnx_OldCompClear returned -0x%X", -return_code);
      return KhiResultCode::FAILURE;
    }
  }

  if (!reset_home_position())
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Failed to reset home position");
    return KhiResultCode::FAILURE;
  }

  // Init command_positions
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    robot_.arms[arm_no].command_positions = krnx_arm_data_[arm_no].home_positions;
    robot_.arms[arm_no].old_command_positions = krnx_arm_data_[arm_no].home_positions;
  }

  // Change monitor speed to 100%.
  constexpr int spd = 100;  // [%]
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    int error_code = 0;
    const int return_code = krnx_SetMonSpeed(robot_.controller_no, arm_no, spd, &error_code);
    if (return_code != KRNX_NOERROR)
    {
      handle_krnx_error("krnx_SetMonSpeed", return_code, error_code, arm_no);
      return KhiResultCode::FAILURE;
    }
  }

  if (!exec_rtc_program())
  {
    return KhiResultCode::FAILURE;
  }

  return KhiResultCode::SUCCESS;
}

/**
 * @brief deactivate
 */
KhiResultCode KhiKrnxDriver::deactivate()
{
  // Add wait time to account for the possiblity of pressing the HOLD button on the TP during
  // ACTIVATE.
  rclcpp::sleep_for(std::chrono::seconds(1));

  if (!hold(true))
  {
    return KhiResultCode::FAILURE;
  }

  if (!kill_program(true))
  {
    return KhiResultCode::FAILURE;
  }

  if (!power_off_motor(true))
  {
    return KhiResultCode::FAILURE;
  }

  return KhiResultCode::SUCCESS;
}

KhiResultCode KhiKrnxDriver::cleanup()
{
  if (!close(true))
  {
    return KhiResultCode::FAILURE;
  }

  return KhiResultCode::SUCCESS;
}

KhiResultCode KhiKrnxDriver::shutdown()
{
  hold(false);

  kill_program(false);

  power_off_motor(false);

  close(false);

  return KhiResultCode::SUCCESS;
}

KhiResultCode KhiKrnxDriver::error()
{
  hold(false);

  kill_program(false);

  power_off_motor(false);

  close(false);

  return KhiResultCode::SUCCESS;
}

/**
 * @brief Read the robot's status.
 * @param cont_no Controller No
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::read()
{
  TKrnxCurMotionDataEx motion_cur[KRNX_MAX_ROBOT];
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    if (arm_no < 0)
    {
      continue;
    }
    if (!get_curmotion_data_ex(robot_.controller_no, arm_no, &motion_cur[arm_no]))
    {
      return false;
    }

    // Set Position
    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      robot_.arms[arm_no].state_positions[jt] = motion_cur[arm_no].ang[jt];
      if (robot_.arms[arm_no].joint_types[jt] == jt_type_prismatic_)
      {
        robot_.arms[arm_no].state_positions[jt] *= MM2M;
      }
    }

    // Set Velocity
    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      robot_.arms[arm_no].state_velocities[jt] = motion_cur[arm_no].ang_vel[jt];
      if (robot_.arms[arm_no].joint_types[jt] == jt_type_prismatic_)
      {
        robot_.arms[arm_no].state_velocities[jt] *= MM2M;
      }
    }

    // Set Effort
    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      robot_.arms[arm_no].state_efforts[jt] = 0;
    }
  }

  // Set F/T sensor data
  if (periodic_data_config_.is_ft_sensor_enabled)
  {
    TKrnxRtExtraData data = {};
    krnx_GetRtCyclicExtraData(robot_.controller_no, &data);
    if (robot_.ft_sensor.enable_n_nm_output)
    {
      robot_.ft_sensor.force_x = data.extra_data[FORCE_X] * robot_.ft_sensor.counter_to_n_ratio;
      robot_.ft_sensor.force_y = data.extra_data[FORCE_Y] * robot_.ft_sensor.counter_to_n_ratio;
      robot_.ft_sensor.force_z = data.extra_data[FORCE_Z] * robot_.ft_sensor.counter_to_n_ratio;
      robot_.ft_sensor.torque_x = data.extra_data[TORQUE_X] * robot_.ft_sensor.counter_to_nm_ratio;
      robot_.ft_sensor.torque_y = data.extra_data[TORQUE_Y] * robot_.ft_sensor.counter_to_nm_ratio;
      robot_.ft_sensor.torque_z = data.extra_data[TORQUE_Z] * robot_.ft_sensor.counter_to_nm_ratio;
    }
    else
    {
      robot_.ft_sensor.force_x = static_cast<double>(data.extra_data[FORCE_X]);
      robot_.ft_sensor.force_y = static_cast<double>(data.extra_data[FORCE_Y]);
      robot_.ft_sensor.force_z = static_cast<double>(data.extra_data[FORCE_Z]);
      robot_.ft_sensor.torque_x = static_cast<double>(data.extra_data[TORQUE_X]);
      robot_.ft_sensor.torque_y = static_cast<double>(data.extra_data[TORQUE_Y]);
      robot_.ft_sensor.torque_z = static_cast<double>(data.extra_data[TORQUE_Z]);
    }
  }

  return true;
}

/**
 * @brief write
 */
bool KhiKrnxDriver::write()
{
  // Calc Compensation amount
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      if (robot_.arms[arm_no].joint_types[jt] == jt_type_prismatic_)
      {
        krnx_arm_data_[arm_no].comp[jt] = static_cast<float>(
          robot_.arms[arm_no].command_positions[jt] * M2MM -
          krnx_arm_data_[arm_no].home_positions[jt]);
      }
      else
      {
        krnx_arm_data_[arm_no].comp[jt] = static_cast<float>(
          robot_.arms[arm_no].command_positions[jt] - krnx_arm_data_[arm_no].home_positions[jt]);
      }
    }
  }

  // Due to slight discrepancies in the cycles of the robot controller and ros2_control, the
  // position command buffer accumulates. Therefore, to prevent the buffer from overflowing, the
  // sending of position commands is skipped when the robot is stationary.
  bool should_decrease_buffer = true;
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    if (krnx_GetRtcBufferLength(robot_.controller_no, arm_no) < KRNX_BUFFER_SIZE)
    {
      should_decrease_buffer = false;
    }
  }
  if (is_position_command_constant() && should_decrease_buffer)
  {
    return true;
  }

  bool is_primed = true;
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    auto return_code = krnx_PrimeRtcCompData(
      robot_.controller_no, arm_no, krnx_arm_data_[arm_no].comp, krnx_arm_data_[arm_no].status);
    if (return_code != KRNX_NOERROR)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "krnx_PrimeRtcCompData -0x%X", -return_code);
      is_primed = false;
    }
  }

  if (!is_primed)
  {
    report_write_error();
    return false;
  }

  const int return_code = krnx_SendRtcCompData(robot_.controller_no, seq_no_);
  if (return_code != KRNX_NOERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "krnx_SendRtcCompData -0x%X", -return_code);
    return false;
  }

  seq_no_++;

  for (auto & arm : robot_.arms)
  {
    arm.old_command_positions = arm.command_positions;
  }

  return true;
}

/**
 * @brief Connect to the controller
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::open() const
{
  RCLCPP_INFO(
    rclcpp::get_logger("khi_hardware"), "Connecting to real controller: %s",
    robot_.ip_address.c_str());

  constexpr int ip_size = 64;
  char ip_address[ip_size] = {};
  TKrnxOpenErrorInfo err_info = {};
  strncpy(ip_address, robot_.ip_address.c_str(), sizeof(ip_address) - 1);
  const int return_code = krnx_OpenSingleProcess(robot_.controller_no, ip_address, &err_info);
  if (return_code == robot_.controller_no)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("khi_hardware"), " Open succeeded. (controller_no:%d)",
      robot_.controller_no);
    return true;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("khi_hardware"), "Open failed. (controller_no:%d)", robot_.controller_no);
  RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "krnx_Open returned -0x%X", -return_code);
  switch (return_code)
  {
    case KRNX_E_BADARGS:
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "The argument for krnx_Open is invalid.");
      break;
    case KRNX_E_NOTSUPPORTED:
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "The currently used version of KRNX does not support the connected robot.");
      break;
    case KRNX_E_DISABLED:
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "The robot setting is invalid. Please check if the KRNX option is enabled.");
      break;
    case KRNX_E_OPEN_NETWORK:
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "'%s' failed (errno=%d port=%d). The following causes may be considered:", err_info.source,
        err_info.err_no, err_info.port);
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), " - Network error");
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), " - Incorrect IP address");
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), " - Issues with port_def.tbl or port_ref.tbl");
      break;
    case KRNX_E_OPEN_CONNECTED_PROCESS:
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Another process (pid=%d) is currently connected to the robot controller."
        "Please terminate the connected process and try again.",
        err_info.pid);
      break;
    case KRNX_E_OPEN_RTLOGIN:
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Failed to open due to high processing load on the robot controller. "
        "Please stop any running robot or PC programs and try opening again. "
        "If the issue persists, power cycle the robot controller and try again.");
      break;
    case KRNX_E_OPEN_ASHANG:
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "The following causes may be considered:");
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        " - The robot controller is frozen. "
        "Please power cycle the robot controller and try opening again.");
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        " - The robot setting is invalid. Please check if the KRNX option is enabled.");
      break;
    case KRNX_E_ALREADYOPENED:
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Already opened in this process.");
      break;
    case KRNX_E_RT_CYCLIC:
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "The software version of the robot controller does not meet the requirements of ROS2.");
      break;
  }
  return false;
}

/**
 * @brief Disonnect from the controller
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::close(const bool need_log) const
{
  const int return_code = krnx_Close(robot_.controller_no);
  if (return_code != KRNX_NOERROR)
  {
    if (need_log)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "krnx_Close returned -0x%X", -return_code);
    }
    return false;
  }

  return true;
}

/**
 * @brief Execute AS Monitor Command.
 */
bool KhiKrnxDriver::exec_monitor_command(
  const int cont_no, const char * cmd, char * buffer, int buffer_sz, int * as_err_code,
  const bool need_log) const
{
  const int return_code = krnx_ExecMon(cont_no, cmd, buffer, buffer_sz, as_err_code);
  if (*as_err_code != 0)
  {
    if (need_log)
    {
      RCLCPP_WARN(rclcpp::get_logger("khi_hardware"), "AS returned %d by %s", *as_err_code, cmd);
    }
    return false;
  }

  if (return_code != KRNX_NOERROR)
  {
    if (need_log)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "%s -0x%X", cmd, -return_code);
    }
    return false;
  }

  return true;
}

/**
 * @brief Determinate if the controller's status meets the conditions for using ROS.
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::has_met_ros_requirements() const
{
  TKrnxCurRobotStatus status;
  bool is_ok = true;

  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    /* Condition Check */
    const int return_code = krnx_GetCurRobotStatus(robot_.controller_no, arm_no, &status);
    if (return_code != KRNX_NOERROR)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "krnx_GetCurRobotStatus returned -0x%X", -return_code);
      is_ok = false;
    }

    if (status.repeat_lamp == OFF)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Please change Robot Controller's TEACH/REPEAT to REPEAT");
      is_ok = false;
    }
    if (status.teach_lock_lamp == ON)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "Please change Robot Controller's TEACH LOCK to OFF");
      is_ok = false;
    }
    if (status.run_lamp == OFF)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "Please change Robot Controller's RUN/HOLD to RUN");
      is_ok = false;
    }
    if ((status.emergency == ON) || (status.system_emergency == ON))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "Please change Robot Controller's EMERGENCY to OFF");
      is_ok = false;
    }
    if (status.protective_stop == ON)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Please change Robot Controller's Protective Stop to OFF");
      is_ok = false;
    }
  }

  return is_ok;
}

bool KhiKrnxDriver::hold(const bool need_log) const
{
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    int error_code = 0;
    const int return_code = krnx_Hold(robot_.controller_no, arm_no, &error_code);
    if (return_code != KRNX_NOERROR)
    {
      if (need_log)
      {
        handle_krnx_error("krnx_Hold", return_code, error_code, arm_no);
      }
      return false;
    }
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  return true;
}

bool KhiKrnxDriver::kill_program(const bool need_log) const
{
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    int error_code = 0;
    const int return_code = krnx_Kill(robot_.controller_no, arm_no, &error_code);
    if (return_code != KRNX_NOERROR)
    {
      if (need_log)
      {
        handle_krnx_error("krnx_Kill", return_code, error_code, arm_no);
      }
      return false;
    }
  }
  return true;
}

/**
 * @brief Determinate if the program is running
 * @param cont_no Controller No
 * @return true Running
 * @return false Not running
 */
bool KhiKrnxDriver::is_program_running() const
{
  bool is_program_running = true;
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    TKrnxCurRobotStatus status;
    const int return_code = krnx_GetCurRobotStatus(robot_.controller_no, arm_no, &status);
    if (return_code != KRNX_NOERROR)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "krnx_GetCurRobotStatus returned -0x%X", -return_code);
      return false;
    }

    if (status.cycle_lamp == OFF)
    {
      is_program_running = false;
    }
  }
  return is_program_running;
}

/**
 * @brief Reset the error.
 * @param cont_no Controller No
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::reset_error() const
{
  // Reset the error if it is in an error state.
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    int error_code = 0;
    int return_code = krnx_Ereset(robot_.controller_no, arm_no, &error_code);
    if (return_code != KRNX_NOERROR)
    {
      handle_krnx_error("krnx_Ereset", return_code, error_code, arm_no);
      return false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(ERROR_RESET_TIME));

    return_code = krnx_GetCurErrorInfo(robot_.controller_no, arm_no, &error_code);
    if (return_code != KRNX_NOERROR)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "krnx_GetCurErrorLamp returned -0x%X", -return_code);
      return false;
    }

    if (error_code != 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Unable to reset error (AS error:%d controller_no:%d arm_no:%d).", error_code,
        robot_.controller_no, arm_no + 1);
      return false;
    }
  }

  return true;
}

/**
 * @brief Turn on the motor power.
 * @param cont_no controller number
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::power_on_motor() const
{
  // Motor Power ON
  int error_code = 0;
  char msg_buf[KRNX_MSGSIZE];
  if (!exec_monitor_command(
        robot_.controller_no, "ZPOW ON", msg_buf, sizeof(msg_buf), &error_code, true))
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Cannot turn on motor power.");
    return false;
  }

  // Wait until motor_lamp turns ON
  const int timeout_msec = 3000;
  const int max_cnt = timeout_msec / robot_.period;
  bool is_motor_lamp_on = false;
  for (int cnt = 0; cnt < max_cnt; cnt++)
  {
    int motor_lamp = OFF;
    if (!get_motor_lamp(motor_lamp, 0, true))
    {
      return false;
    }
    if (motor_lamp == ON)
    {
      is_motor_lamp_on = true;
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(robot_.period)));
  }

  if (!is_motor_lamp_on)
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Cannot turn on motor power.");
    return false;
  }

  return true;
}

/**
 * @brief Turn off the motor power.
 * @param cont_no controller number
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::power_off_motor(const bool need_log) const
{
  // Motor Power ON
  int error_code = 0;
  char msg_buf[KRNX_MSGSIZE];
  if (!exec_monitor_command(
        robot_.controller_no, "ZPOW OFF", msg_buf, sizeof(msg_buf), &error_code, need_log))
  {
    if (need_log)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Cannot turn off motor power.");
    }
    return false;
  }

  // Wait until motor_lamp turns OFF
  const int timeout_msec = 3000;
  const int max_cnt = timeout_msec / robot_.period;
  bool is_motor_lamp_off = false;
  for (int cnt = 0; cnt < max_cnt; cnt++)
  {
    int motor_lamp = ON;
    if (!get_motor_lamp(motor_lamp, 0, need_log))
    {
      return false;
    }
    if (motor_lamp == OFF)
    {
      is_motor_lamp_off = true;
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(robot_.period)));
  }

  if (!is_motor_lamp_off)
  {
    if (need_log)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Cannot turn off motor power.");
    }
    return false;
  }
  return true;
}

/**
 * @brief Get motor lamp
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::get_motor_lamp(int & motor_lamp, const int arm_no, const bool need_log) const
{
  TKrnxCurRobotStatus status;
  const int return_code = krnx_GetCurRobotStatus(robot_.controller_no, arm_no, &status);
  if (return_code != KRNX_NOERROR)
  {
    if (need_log)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"), "krnx_GetCurRobotStatus returned -0x%X", -return_code);
    }
    return false;
  }
  motor_lamp = status.motor_lamp;
  return true;
}

/**
 * @brief Move to the Home Position.
 * @param cont_no Controller No
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::exec_rtc_program() const
{
  const double timeout_sec_th = 5.0; /* 5 sec */
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    // Execute the program for RTC.
    std::string program = "rb_rtc" + std::to_string(arm_no + 1);
    int error_code = 0;
    const int return_code =
      krnx_Execute(robot_.controller_no, arm_no, program.c_str(), 1, 0, &error_code);
    if (return_code != KRNX_NOERROR)
    {
      handle_krnx_error("krnx_Execute", return_code, error_code, arm_no);
      return false;
    }

    // Wait until rtc_active becomes ON
    double timeout_sec_cnt = 0;
    while (true)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(robot_.period)));

      timeout_sec_cnt += robot_.period * MSEC2SEC;
      if (timeout_sec_cnt > timeout_sec_th)
      {
        RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Failed to activate: timeout");
        return false;
      }

      // Detect errors caused by executing the robot program.
      if (is_error())
      {
        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait until the error code is updated.
        krnx_GetCurErrorInfo(robot_.controller_no, arm_no, &error_code);
        RCLCPP_ERROR(
          rclcpp::get_logger("khi_hardware"), "AS ERROR controller_no:%d arm_no:%d error_code:%d",
          robot_.controller_no, arm_no + 1, error_code);
        return false;
      }

      TKrnxCurRobotStatus status;
      const int return_code = krnx_GetCurRobotStatus(robot_.controller_no, arm_no, &status);
      if (return_code != KRNX_NOERROR)
      {
        RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Failed to exec krnx_GetCurRobotStatus.");
        return false;
      }
      if (status.rtc_active == OFF)
      {
        continue;
      }
      break;
    }
  }

  return true;
}

/**
 * @brief Move to the Home Position.
 * @param cont_no Controller No
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::is_configuration_valid() const
{
  int arm_num = static_cast<int>(robot_.arms.size());
  if (arm_num <= 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "Invalid robot size. Confirm number of robot arm.");
    return false;
  }

  for (int arm_no = 0; arm_no < arm_num; arm_no++)
  {
    // Convert the argument robot name to uppercase.
    auto ros_robot_name = robot_.name;
    for (char & c : ros_robot_name)
    {
      c = std::toupper(c);
    }

    // Verify that the robot name matches.
    constexpr int name_size = 64;
    char robot_name[name_size] = {0};
    krnx_GetRobotName(robot_.controller_no, arm_no, robot_name);
    if (strncmp(robot_name, ros_robot_name.c_str(), ros_robot_name.size()) != 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "ROS Robot:%s does not match AS:%s. Match the robot model on ROS and robot.",
        ros_robot_name.c_str(), robot_name);
      return false;
    }

    // Verify the correct number of joint.
    int error_code = 0;
    char cmd[KRNX_CMD_SIZE] = {0};
    char msg_buf[KRNX_MSGSIZE];
    snprintf(cmd, sizeof(cmd), "TYPE SYSDATA(ZROB.NOWAXIS,%d)", arm_no + 1);
    if (!exec_monitor_command(
          robot_.controller_no, cmd, msg_buf, sizeof(msg_buf), &error_code, true))
    {
      return false;
    }
    constexpr int decimal = 10;
    char * end_ptr = nullptr;
    auto jt_num_tmp = strtol(msg_buf, &end_ptr, decimal);
    if ((jt_num_tmp == LONG_MAX) || (jt_num_tmp == LONG_MIN) || (end_ptr == msg_buf))
    {
      return false;
    }
    int jt_num = static_cast<int>(jt_num_tmp);

    if (robot_.arms[arm_no].joint_num != jt_num)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("khi_hardware"), "ROS JT:%d does not match AS:%d",
        robot_.arms[arm_no].joint_num, jt_num);
    }
  }

  return true;
}

/**
 * @brief Verify if the software version of the robot controller meets the requirements of ROS2.
 * @return true
 * @return false
 */
bool KhiKrnxDriver::is_robot_software_version_valid() const
{
  // Verify if the software supports RTC_CTL.
  int error_code = 0;
  char msg_buf[KRNX_MSGSIZE] = {};
  char expected_error_code[] = " ^(P0109)";
  krnx_ExecMon(robot_.controller_no, "RTC_CTL", msg_buf, sizeof(msg_buf), &error_code);
  if (strncmp(msg_buf, expected_error_code, sizeof(expected_error_code) - 1) == 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"),
      "The software version of the robot controller does not meet the requirements of ROS2.");
    return false;
  }

  return true;
}

bool KhiKrnxDriver::get_curmotion_data_ex(
  const int cont_no, const int robot_no, TKrnxCurMotionDataEx * p_motion_data) const
{
  const int return_code = krnx_GetCurMotionDataEx(cont_no, robot_no, p_motion_data);
  if (return_code != KRNX_NOERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "krnx_GetCurMotionData returned -0x%X", -return_code);
    return false;
  }

  return true;
}

/**
 * @brief Output errors when writing.
 */
void KhiKrnxDriver::report_write_error() const
{
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    float old_comp[KRNX_MAXAXES] = {};
    float spd_limits[KRNX_MAXAXES] = {};
    krnx_GetRtcCompData(robot_.controller_no, arm_no, old_comp);
    krnx_GetRtcCompLimit(robot_.controller_no, arm_no, spd_limits);

    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      bool is_prismatic = (robot_.arms[arm_no].joint_types[jt] == jt_type_prismatic_);
      auto cmd = krnx_arm_data_[arm_no].home_positions[jt] + krnx_arm_data_[arm_no].comp[jt];
      auto old_cmd = krnx_arm_data_[arm_no].home_positions[jt] + old_comp[jt];
      auto actual_pos = robot_.arms[arm_no].state_positions[jt];

      cmd *= is_prismatic ? MM2M : RAD2DEG;
      old_cmd *= is_prismatic ? MM2M : RAD2DEG;
      if (!is_prismatic)
      {
        actual_pos *= RAD2DEG;
      }

      if (
        (krnx_arm_data_[arm_no].status[jt] & KRNX_POS_UPPER_LIMIT_ERR) ||
        (krnx_arm_data_[arm_no].status[jt] & KRNX_POS_LOWER_LIMIT_ERR))
      {
        std::string msg = "A commanded position exceeding the operating range was sent. ";
        msg += ("( JT" + std::to_string(jt + 1));
        msg += (" cmd:" + std::to_string(cmd));
        msg += is_prismatic ? "[m]" : "[deg]";
        msg += (" actual_pos:" + std::to_string(actual_pos));
        msg += is_prismatic ? "[m] )" : "[deg] )";
        RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), msg.c_str());
      }

      if (krnx_arm_data_[arm_no].status[jt] & KRNX_SPD_LIMIT_ERR)
      {
        int buf_size = krnx_GetRtcBufferLength(robot_.controller_no, arm_no);
        auto spd = fabs(krnx_arm_data_[arm_no].comp[jt] - old_comp[jt]) * (1000.0 / robot_.period);
        auto spd_limit = spd_limits[jt] * (1000.0 / robot_.period);
        spd *= is_prismatic ? MM2M : RAD2DEG;
        spd_limit *= is_prismatic ? MM2M : RAD2DEG;

        std::string msg = "A commanded position exceeding the speed limit was sent. ";
        msg += ("( JT" + std::to_string(jt + 1));
        msg += (" cmd:" + std::to_string(cmd));
        msg += is_prismatic ? "[m]" : "[deg]";
        msg += (" old_cmd:" + std::to_string(old_cmd));
        msg += is_prismatic ? "[m]" : "[deg]";
        msg += (" cmd_spd:" + std::to_string(spd));
        msg += is_prismatic ? "[m/s]" : "[deg/s]";
        msg += (" spd_limit:" + std::to_string(spd_limit));
        msg += is_prismatic ? "[m/s]" : "[deg/s]";
        msg += (" actual_pos:" + std::to_string(actual_pos));
        msg += is_prismatic ? "[m]" : "[deg]";
        msg += (" buf_size:" + std::to_string(buf_size) + ")");
        RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), msg.c_str());
      }
    }
  }
}

/**
 * @brief Load RealTimeControl program
 * @return true Success
 * @return false Failure
 * @memberof KhiKrnxDriver
 */
bool KhiKrnxDriver::load_rtc_program() const
{
  auto report_error = []
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "Failed to load RTC program");
    return false;
  };

  if (is_program_running())
  {
    if (!hold(true))
    {
      return report_error();
    }
  }

  if (!power_off_motor(true))
  {
    return report_error();
  }

  if (!kill_program(true))
  {
    return report_error();
  }

  constexpr int path_size = 128;
  char file_path[path_size] = {0};
  char tmplt[] = "/tmp/khi_robot-rtc_param-XXXXXX";
  auto fd = mkstemp(tmplt);
  FILE * fp = fdopen(fd, "w");
  if (fp != nullptr)
  {
    char fd_path[path_size] = {0};

    /* retrieve path */
    snprintf(fd_path, sizeof(fd_path), "/proc/%d/fd/%d", getpid(), fd);
    const ssize_t rsize = readlink(fd_path, file_path, sizeof(file_path));
    if (rsize < 0)
    {
      return report_error();
    }

    /* RTC program (for arm1) */
    fprintf(fp, ".PROGRAM rb_rtc1()\n");
    fprintf(fp, "  RTC_SW 1: ON\n");
    fprintf(fp, "1 RTC_CTL\n");
    fprintf(fp, "  GOTO 1\n");
    fprintf(fp, "  RTC_SW 1: OFF\n");
    fprintf(fp, ".END\n");
    fclose(fp);
  }
  else
  {
    return report_error();
  }

  auto return_code = krnx_Load(robot_.controller_no, file_path);
  if (return_code != KRNX_NOERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "krnx_Load returned -0x%X %s", -return_code, file_path);
    return report_error();
  }

  unlink(file_path);

  return true;
}

bool KhiKrnxDriver::reset_home_position()
{
  TKrnxCurMotionDataEx motion_data = {};

  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    /* Driver */
    if (!get_curmotion_data_ex(robot_.controller_no, arm_no, &motion_data))
    {
      return false;
    }

    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      krnx_arm_data_[arm_no].home_positions[jt] = static_cast<double>(motion_data.ang[jt]);
      if (robot_.arms[arm_no].joint_types[jt] == jt_type_prismatic_)
      {
        krnx_arm_data_[arm_no].home_positions[jt] /= M2MM;
      }
    }
  }

  return true;
}

/**
 * @brief Get signal status.
 */
void KhiKrnxDriver::get_signal_srv_cb(
  const khi_msgs::srv::GetSignal::Request::SharedPtr & req,
  const khi_msgs::srv::GetSignal::Response::SharedPtr & resp) const
{
  resp->success = false;

  TKrnxIoInfo io;
  const int return_code = krnx_GetCurIoInfo(robot_.controller_no, &io);
  if (return_code != KRNX_NOERROR)
  {
    return;
  }

  for (auto sig_no : req->signal_numbers)
  {
    if (sig_no >= 1 && sig_no <= KHI_MAX_SIG_SIZE)
    {
      // DO
      bool is_on = io.io_do[(sig_no - 1) / 8] & (1 << (sig_no - 1) % 8);
      resp->is_on.push_back(is_on);
    }
    else if (sig_no >= 1001 && sig_no <= 1000 + KHI_MAX_SIG_SIZE)
    {
      // DI
      int sig_no_tmp = sig_no - 1000;
      bool is_on = io.io_di[(sig_no_tmp - 1) / 8] & (1 << (sig_no_tmp - 1) % 8);
      resp->is_on.push_back(is_on);
    }
    else if (sig_no >= 2001 && sig_no <= 2000 + KHI_MAX_SIG_SIZE)
    {
      // INTERNAL
      int sig_no_tmp = sig_no - 2000;
      bool is_on = io.internal[(sig_no_tmp - 1) / 8] & (1 << (sig_no_tmp - 1) % 8);
      resp->is_on.push_back(is_on);
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Signal Number(%d) is incorrect.(DO:1~%d, DI:1000~%d, INTERNAL:2001~%d)", sig_no,
        KHI_MAX_SIG_SIZE, KHI_MAX_SIG_SIZE + 1000, KHI_MAX_SIG_SIZE + 2000);
      return;
    }
  }
  resp->success = true;
}

/**
 * @brief Set signal status.
 */
void KhiKrnxDriver::set_signal_srv_cb(
  const khi_msgs::srv::SetSignal::Request::SharedPtr & req,
  const khi_msgs::srv::SetSignal::Response::SharedPtr & resp) const
{
  resp->success = false;

  // Check request
  if (req->signal_numbers.size() != req->is_on.size())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"),
      "The counts of signal_numbers and is_on do not match. Please ensure they are "
      "equal.(signal_numbers size:%d, is_on size:%d)",
      static_cast<int>(req->signal_numbers.size()), static_cast<int>(req->is_on.size()));
    return;
  }

  // Make command
  std::string as_cmd = "SIGNAL ";
  for (int sig_no = 0; sig_no < static_cast<int>(req->signal_numbers.size()); sig_no++)
  {
    int sig = (req->is_on[sig_no]) ? req->signal_numbers[sig_no] : -req->signal_numbers[sig_no];
    as_cmd += std::to_string(sig);
    as_cmd += ", ";
  }
  as_cmd.erase(as_cmd.length() - 2);

  // Exec command
  int error_code = 0;
  char msg_buf[KRNX_MSGSIZE];
  if (!exec_monitor_command(
        robot_.controller_no, as_cmd.c_str(), msg_buf, sizeof(msg_buf), &error_code, true))
  {
    resp->error_code = error_code;
    std::string cmd = "TYPE $ERROR(" + std::to_string(error_code) + ")";
    char msg[KRNX_MSGSIZE];
    exec_monitor_command(robot_.controller_no, cmd.c_str(), msg, sizeof(msg), &error_code, true);
    resp->error_msg = convert_to_utf8(msg);
    return;
  }

  resp->success = true;
}

void KhiKrnxDriver::exec_khi_command_srv_cb(
  const khi_msgs::srv::ExecKhiCommand::Request::SharedPtr & req,
  const khi_msgs::srv::ExecKhiCommand::Response::SharedPtr & resp) const
{
  resp->success = true;

  // Exec command
  int error_code = 0;
  char return_msg[KRNX_MSGSIZE];
  const int return_code = krnx_ExecMon(
    robot_.controller_no, req->command.c_str(), return_msg, sizeof(return_msg), &error_code);

  if (return_code != KRNX_NOERROR)
  {
    std::stringstream ss;
    ss << std::hex << -return_code;
    resp->krnx_err = "-0x" + ss.str();
    resp->success = false;
  }

  resp->return_message = convert_to_utf8(return_msg);

  if (error_code != 0)
  {
    resp->error_code = error_code;
    std::string cmd = "TYPE $ERROR(" + std::to_string(error_code) + ")";
    char msg[KRNX_MSGSIZE];
    exec_monitor_command(robot_.controller_no, cmd.c_str(), msg, sizeof(msg), &error_code, true);
    resp->error_msg = convert_to_utf8(msg);
    return;
  }
}

void KhiKrnxDriver::reset_error_srv_cb(
  const khi_msgs::srv::ResetError::Request::SharedPtr & /*req*/,
  const khi_msgs::srv::ResetError::Response::SharedPtr & resp, const int arm_no) const
{
  resp->success = false;

  int error_code = 0;
  int return_code = krnx_Ereset(robot_.controller_no, arm_no, &error_code);
  if (return_code != KRNX_NOERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "krnx_Ereset returned -0x%X", -return_code);
    return;
  }

  // Check if the error has reocuured after resetting the error
  rclcpp::sleep_for(std::chrono::milliseconds(ERROR_RESET_TIME));
  int error_lamp = 0;
  return_code = krnx_GetCurErrorLamp(robot_.controller_no, arm_no, &error_lamp);
  if (return_code != KRNX_NOERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "krnx_GetCurErrorLamp returned -0x%X", -return_code);
    return;
  }

  // Record error information
  if (error_lamp == ON)
  {
    TKrnxErrorList list;
    const int return_code = krnx_GetCurErrorList(robot_.controller_no, &list);
    if (return_code != KRNX_NOERROR)
    {
      return;
    }

    for (auto code : list.error_code)
    {
      if (code != 0)
      {
        resp->error_codes.push_back(code);
      }
    }
    for (const auto & msg : list.error_msg)
    {
      if (msg[0] != '\0')
      {
        std::string str_msg = convert_to_utf8(msg);
        resp->error_msgs.push_back(str_msg);
      }
    }
    return;
  }

  resp->success = true;
}

bool KhiKrnxDriver::is_error() const
{
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    int error_lamp = 0;
    const int return_code = krnx_GetCurErrorLamp(robot_.controller_no, arm_no, &error_lamp);
    if (return_code != KRNX_NOERROR)
    {
      return false;
    }
    if (error_lamp == ON)
    {
      return true;
    }
  }
  return false;
}

void KhiKrnxDriver::convert_error_code(std::string & str_error_code, int error_code) const
{
  char level[2];
  krnx_ConvertErrorCode(&error_code, level);
  str_error_code = std::string(level) + std::to_string(error_code);
}

bool KhiKrnxDriver::get_error_info(
  std::vector<int> & error_codes, std::vector<std::string> & error_msgs) const
{
  TKrnxErrorList list;
  const int return_code = krnx_GetCurErrorList(robot_.controller_no, &list);
  if (return_code != KRNX_NOERROR)
  {
    return false;
  }

  for (auto code : list.error_code)
  {
    if (code != 0)
    {
      error_codes.push_back(code);
    }
  }
  for (const auto & msg : list.error_msg)
  {
    if (msg[0] != '\0')
    {
      std::string str_msg = convert_to_utf8(msg);
      error_msgs.push_back(str_msg);
    }
  }

  return true;
}

/**
 * @brief Checks if command values can be written.
 * @return true if writable
 * @return false if not
 * @memberof KhiKrnxDriver
 */
bool KhiKrnxDriver::is_writable()
{
  const int cont_no = robot_.controller_no;

  // Check if command values can be written to the robot.
  bool is_writable = true;
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    TKrnxCurRobotStatus status = {};
    krnx_GetCurRobotStatus(cont_no, arm_no, &status);
    if (status.rtc_active == 0)
    {
      is_writable = false;
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "The robot controller cannot be operated with external command values. (controller_no:%d "
        "arm_no:%d)",
        cont_no, arm_no + 1);
    }
  }

  // Display the reasons for write failure.
  if (!is_writable)
  {
    if (!is_program_running())
    {
      for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
      {
        std::string msg = "Program 'rb_rtc" + std::to_string(arm_no + 1) + "' is not running.";
        RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), msg.c_str());
      }
    }

    // Check if an error has occurred.
    for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
    {
      int error_lamp = 0;
      krnx_GetCurErrorLamp(cont_no, arm_no, &error_lamp);
      if (error_lamp != 0)
      {
        int error_code = 0;
        krnx_GetCurErrorInfo(cont_no, arm_no, &error_code);
        RCLCPP_ERROR(
          rclcpp::get_logger("khi_hardware"), "AS ERROR controller_no:%d arm_no:%d error_code:%d",
          cont_no, arm_no + 1, error_code);
      }
    }

    // Investigate the cause of the stop.
    has_met_ros_requirements();

    return false;
  }

  return true;
}

/**
 * @brief Get the actual current value
 * @param actual_current actual current value
 * @return true Success
 * @return false Failure
 */
bool KhiKrnxDriver::get_actual_current(std::vector<float> & actual_current) const
{
  if (!periodic_data_config_.is_actual_current_enabled)
  {
    return false;
  }

  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    TKrnxCurMotionDataEx data = {};
    const int return_code = krnx_GetCurMotionDataEx(robot_.controller_no, arm_no, &data);
    if (return_code != KRNX_NOERROR)
    {
      return false;
    }

    for (auto jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      actual_current.push_back(data.cur[jt]);
    }
  }

  return true;
}

/**
 * \brief Check the language settings of the robot controller.
 * \return true Success
 * \return false Failure
 */
bool KhiKrnxDriver::chk_language()
{
  std::string cmd = "TYPE SYSDATA(LANGUAGE)";
  char msg[KRNX_MSGSIZE];
  int err_code = 0;
  if (!exec_monitor_command(robot_.controller_no, cmd.c_str(), msg, sizeof(msg), &err_code, true))
  {
    return false;
  }
  is_japanese_ = (std::stoi(std::string(msg)) == 1);
  is_chinese_ = (std::stoi(std::string(msg)) == 6);
  is_korean_ = (std::stoi(std::string(msg)) == 7);

  return true;
}

/**
 * \brief Set the timeout duration for the KRNX API.
 * \param req request
 * \param resp responce
 * \memberof KhiKrnxDriver
 */
void KhiKrnxDriver::set_timeout_srv_cb(
  const khi_msgs::srv::SetTimeout::Request::SharedPtr & req,
  const khi_msgs::srv::SetTimeout::Response::SharedPtr & resp) const
{
  resp->success = true;

  int return_code = krnx_SetAuxApiTimeoutPeriod(robot_.controller_no, req->aux_timeout);

  if (return_code != KRNX_NOERROR)
  {
    std::stringstream ss;
    ss << std::hex << -return_code;
    resp->error = "-0x" + ss.str();
    resp->success = false;
  }
}

/**
 * \brief Checks if communicating with the robot controller
 * \return true : If communicating with the robot controller
 * \return false : If failed to communicate with the robot controller
 * \memberof KhiKrnxDriver
 */
bool KhiKrnxDriver::is_communicating() const
{
  bool is_communicating = false;
  krnx_GetConnectionStatus(robot_.controller_no, &is_communicating);
  return is_communicating;
}

/**
 * @brief Check if the position command has remained constant
 * @return true if the position command has not changed.
 * @return false if the position command has changed.
 * \memberof KhiKrnxDriver
 */
bool KhiKrnxDriver::is_position_command_constant()
{
  constexpr int thresh_cnt = 10;

  bool is_constant = true;
  for (const auto & arm : robot_.arms)
  {
    for (int jt = 0; jt < arm.joint_num; jt++)
    {
      double diff = fabs(arm.command_positions[jt] - arm.old_command_positions[jt]);
      if (diff > __FLT_EPSILON__)
      {
        is_constant = false;
        break;
      }
    }
  }
  cmd_constant_cnt_ = is_constant ? (cmd_constant_cnt_ + 1) : 0;

  return (cmd_constant_cnt_ > thresh_cnt);
}

/**
 * @brief Warns of robot anomalies
 * \memberof KhiKrnxDriver
 */
void KhiKrnxDriver::monitor_robot_health()
{
  // Display a warning about the number of buffed position commands.
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    if (krnx_GetRtcBufferLength(robot_.controller_no, arm_no) > KRNX_BUFFER_SIZE_THRESH)
    {
      rtc_buffer_thresh_exceed_cnt_ += 1;
    }
    else
    {
      rtc_buffer_thresh_exceed_cnt_ = 0;
    }

    if (rtc_buffer_thresh_exceed_cnt_ > KRNX_BUFFER_WARNING_INTERVAL)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("khi_hardware"),
        "The number of buffered position commands is too high. Please pause the robot and then "
        "resumed its operation.");
      rtc_buffer_thresh_exceed_cnt_ = 0;
    }
  }

  // Display a warning if current saturation is detected multiple times in a short duration.
  constexpr int duration = 100;
  constexpr int err_thresh = 3;
  static bool is_saturated[KRNX_MAX_ROBOT][KRNX_MAXAXES][duration] = {};
  static unsigned int cnt = 0;
  for (int arm_no = 0; arm_no < static_cast<int>(robot_.arms.size()); arm_no++)
  {
    TKrnxCurMotionDataEx data[KRNX_MAX_ROBOT];
    if (!get_curmotion_data_ex(robot_.controller_no, arm_no, &data[arm_no]))
    {
      continue;
    }

    for (int jt = 0; jt < robot_.arms[arm_no].joint_num; jt++)
    {
      is_saturated[arm_no][jt][cnt] = (data->cur_sat[jt] >= 1.0);

      int err_cnt = 0;
      for (auto is_sat : is_saturated[arm_no][jt])
      {
        if (is_sat)
        {
          err_cnt++;
        }
      }
      if (err_cnt > err_thresh)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("khi_hardware"),
          "The current is saturated. Please reduce the acceleration or change the motion. [JT%d "
          "%f]",
          jt + 1, data->cur_sat[jt]);
        for (auto & is_sat : is_saturated[arm_no][jt])
        {
          is_sat = false;
        }
      }
    }
  }
  cnt++;
  cnt %= duration;
}

/**
 * @brief Convert to UTF-8
 * @param input Input string
 * @return std::string Output string
 * @memberof KhiKrnxDriver
 */
std::string KhiKrnxDriver::convert_to_utf8(const char * input) const
{
  if (is_japanese_)
  {
    return convert_encoding(input, "UTF-8", "SHIFT_JIS");
  }
  if (is_chinese_)
  {
    return convert_encoding(input, "UTF-8", "GBK");
  }
  if (is_korean_)
  {
    return convert_encoding(input, "UTF-8", "EUC-KR");
  }
  return std::string(input);
}

/**
 * @brief Handles error checking and logging for KRNX API calls.
 * @param krnx_api Name of the KRNX API function being called
 * @param krnx_return_code Return code from the KRNX API call
 * @param as_err_code AS error
 * @param arm_no arm number
 * @memberof KhiKrnxDriver
 */
void KhiKrnxDriver::handle_krnx_error(
  const std::string & krnx_api, const int krnx_return_code, const int as_err_code,
  const int arm_no) const
{
  if (krnx_return_code == KRNX_E_ASERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"),
      "%s returned -0x%X. (AS error:%d controller_no:%d arm_no:%d)", krnx_api.c_str(),
      -krnx_return_code, as_err_code, robot_.controller_no, arm_no + 1);
  }
  else if (krnx_return_code != KRNX_NOERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("khi_hardware"), "%s returned -0x%X. (controller_no:%d arm_no:%d)",
      krnx_api.c_str(), -krnx_return_code, robot_.controller_no, arm_no + 1);
  }
}

/**
 * @brief Set the type of information that the robot periodically retrieves.
 */
void KhiKrnxDriver::set_periodic_data_config() const
{
  u_int16_t kind = KRNX_CYC_KIND_ANGLE | KRNX_CYC_KIND_ANGLE_REF | KRNX_CYC_KIND_ERROR |
                   KRNX_CYC_KIND_CURRENT_SAT | KRNX_CYC_KIND_ANGLE_VEL | KRNX_CYC_KIND_ROBOT_STATUS;
  if (periodic_data_config_.is_actual_current_enabled)
  {
    kind |= KRNX_CYC_KIND_CURRENT;
  }
  if (periodic_data_config_.is_actual_encorder_enabled)
  {
    kind |= KRNX_CYC_KIND_ENCORDER;
  }
  if (periodic_data_config_.is_command_current_enabled)
  {
    kind |= KRNX_CYC_KIND_CURRENT_REF;
  }
  if (periodic_data_config_.is_command_encorder_enabled)
  {
    kind |= KRNX_CYC_KIND_ENCORDER_REF;
  }
  if (periodic_data_config_.is_tcp_info_enabled)
  {
    kind |= KRNX_CYC_KIND_XYZOAT;
  }
  if (periodic_data_config_.is_external_signal_enabled)
  {
    kind |= KRNX_CYC_KIND_SIG_EXTERNAL;
  }
  if (periodic_data_config_.is_internal_signal_enabled)
  {
    kind |= KRNX_CYC_KIND_SIG_INTERNAL;
  }
  if (periodic_data_config_.is_ft_sensor_enabled)
  {
    kind |= KRNX_CYC_KIND_EXTRA_DATA;
  }
  krnx_SetRtCyclicDataKind(robot_.controller_no, kind);
}

/**
 * @brief Executes the RDT command SetSoftwareBias on the ATI F/T sensor connected to the robot
 * controller.
 * @param req request
 * @param resp responce
 * @memberof KhiKrnxDriver
 */
void KhiKrnxDriver::set_ati_software_bias_srv_cb(
  const khi_msgs::srv::SetATISoftwareBias::Request::SharedPtr & /*req*/,
  const khi_msgs::srv::SetATISoftwareBias::Response::SharedPtr & resp) const
{
  resp->success = true;

  int error_code = 0;
  int return_code = krnx_SetATISoftwareBias(robot_.controller_no, &error_code);

  if (return_code != KRNX_NOERROR)
  {
    std::stringstream ss;
    ss << std::hex << -return_code;
    resp->krnx_err = "-0x" + ss.str();
    resp->success = false;
  }

  if (error_code != 0)
  {
    resp->error_code = error_code;
    std::string cmd = "TYPE $ERROR(" + std::to_string(error_code) + ")";
    char msg[KRNX_MSGSIZE];
    exec_monitor_command(robot_.controller_no, cmd.c_str(), msg, sizeof(msg), &error_code, true);
    resp->error_msg = convert_to_utf8(msg);
    resp->success = false;
  }
}

/**
 * @brief Switch the output unit of the values obtained from the FT sensor (Counter [default] ⇔ N,
 * Nm)
 * @param req request
 * @param resp responce
 * @memberof KhiKrnxDriver
 */
void KhiKrnxDriver::change_ft_output_mode_srv_cb(
  const khi_msgs::srv::ChangeFTOutputMode::Request::SharedPtr & req,
  const khi_msgs::srv::ChangeFTOutputMode::Response::SharedPtr & /*resp*/)
{
  robot_.ft_sensor.enable_n_nm_output = req->enable_n_nm_output;
  robot_.ft_sensor.counter_to_n_ratio = req->counter_to_n_ratio;
  robot_.ft_sensor.counter_to_nm_ratio = req->counter_to_nm_ratio;
}
}  // namespace khi_hardware
