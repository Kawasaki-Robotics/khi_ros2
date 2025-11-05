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

#ifndef KHI_HARDWARE__KHI_SERVICE_HPP_
#define KHI_HARDWARE__KHI_SERVICE_HPP_

#include <memory>

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
class KhiDriver;

class KhiService
{
public:
  KhiService() = default;
  ~KhiService();
  void start(KhiDriver & driver);

private:
  void service_loop(KhiDriver & driver);
  bool should_stop_service_ = false;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<khi_msgs::srv::GetSignal>::SharedPtr get_signal_service_{};
  rclcpp::Service<khi_msgs::srv::SetSignal>::SharedPtr set_signal_service_{};
  rclcpp::Service<khi_msgs::srv::ExecKhiCommand>::SharedPtr khi_cmd_service_{};
  rclcpp::Service<khi_msgs::srv::ResetError>::SharedPtr reset_error_service_{};
  rclcpp::Service<khi_msgs::srv::SetTimeout>::SharedPtr set_timeout_service_{};
  rclcpp::Service<khi_msgs::srv::SetATISoftwareBias>::SharedPtr set_ati_software_bias_service_{};
  rclcpp::Service<khi_msgs::srv::ChangeFTOutputMode>::SharedPtr change_ft_output_mode_service_{};
};

}  // namespace khi_hardware

#endif  // KHI_HARDWARE__KHI_SERVICE_HPP_
