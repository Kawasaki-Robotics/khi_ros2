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

#include "khi_hardware/khi_service.hpp"
#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
KhiService::~KhiService() { should_stop_service_ = true; }

void KhiService::start(KhiDriver & driver)
{
  auto loop = [&]() { service_loop(driver); };
  std::thread spin_thread(loop);
  spin_thread.detach();
}

void KhiService::service_loop(KhiDriver & driver)
{
  if (node_ == nullptr)
  {
    std::string node_namespace =
      "/khi_controller" + std::to_string(driver.get_robot().controller_no);
    node_ = rclcpp::Node::make_shared("khi_service", node_namespace);
  }

  auto get_signal = [&driver](
                      const khi_msgs::srv::GetSignal::Request::SharedPtr & req,
                      const khi_msgs::srv::GetSignal::Response::SharedPtr & resp)
  { driver.get_signal_srv_cb(req, resp); };
  get_signal_service_ = node_->create_service<khi_msgs::srv::GetSignal>("~/get_signal", get_signal);

  auto set_signal = [&driver](
                      const khi_msgs::srv::SetSignal::Request::SharedPtr & req,
                      const khi_msgs::srv::SetSignal::Response::SharedPtr & resp)
  { driver.set_signal_srv_cb(req, resp); };
  set_signal_service_ = node_->create_service<khi_msgs::srv::SetSignal>("~/set_signal", set_signal);

  auto exec_khi_command = [&driver](
                            const khi_msgs::srv::ExecKhiCommand::Request::SharedPtr & req,
                            const khi_msgs::srv::ExecKhiCommand::Response::SharedPtr & resp)
  { driver.exec_khi_command_srv_cb(req, resp); };
  khi_cmd_service_ =
    node_->create_service<khi_msgs::srv::ExecKhiCommand>("~/exec_khi_command", exec_khi_command);

  auto reset_error = [&driver](
                       const khi_msgs::srv::ResetError::Request::SharedPtr & req,
                       const khi_msgs::srv::ResetError::Response::SharedPtr & resp)
  { driver.reset_error_srv_cb(req, resp, 0); };
  reset_error_service_ =
    node_->create_service<khi_msgs::srv::ResetError>("~/reset_error", reset_error);

  auto set_timeout = [&driver](
                       const khi_msgs::srv::SetTimeout::Request::SharedPtr & req,
                       const khi_msgs::srv::SetTimeout::Response::SharedPtr & resp)
  { driver.set_timeout_srv_cb(req, resp); };
  set_timeout_service_ =
    node_->create_service<khi_msgs::srv::SetTimeout>("~/set_timeout", set_timeout);

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiService Start");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  while (!should_stop_service_)
  {
    exec.spin_some();
  }

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiService STOP");
}
}  // namespace khi_hardware
