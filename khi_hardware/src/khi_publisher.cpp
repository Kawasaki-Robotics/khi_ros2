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

#include "khi_hardware/khi_publisher.hpp"
#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_msgs/msg/actual_current.hpp"
#include "khi_msgs/msg/error_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
KhiPublisher::~KhiPublisher()
{
  should_stop_publisher_ = true;
  if (event_thread_.joinable())
  {
    event_thread_.join();
  };
  if (schedule_thread_.joinable())
  {
    schedule_thread_.join();
  };
  error_info_publisher_.reset();
  actual_current_publisher_.reset();
  timer_.reset();
  node_.reset();
}

void KhiPublisher::start(const KhiDriver & driver)
{
  if (node_ == nullptr)
  {
    std::string node_namespace =
      "/khi_controller" + std::to_string(driver.get_robot().controller_no);
    node_ = rclcpp::Node::make_shared("khi_publisher", node_namespace);
  }

  auto event = [&]() { publish_on_event(driver); };
  event_thread_ = std::thread(event);

  auto schedule = [&]() { publish_on_schedule(driver); };
  schedule_thread_ = std::thread(schedule);
}

void KhiPublisher::report_error(const KhiDriver & driver)
{
  // Determine if error repoting should be re-enabled.
  if (should_stop_error_reporting_ && driver.is_error())
  {
    std::vector<int> error_codes;
    std::vector<std::string> error_msgs;
    driver.get_error_info(error_codes, error_msgs);

    if (error_codes.size() != old_error_codes_.size())
    {
      should_stop_error_reporting_ = false;
    }
    else
    {
      for (int i = 0; i < static_cast<int>(error_codes.size()); i++)
      {
        if (error_codes[i] != old_error_codes_[i])
        {
          should_stop_error_reporting_ = false;
        }
      }
    }

    if (!should_stop_error_reporting_)
    {
      old_error_codes_.resize(0);
    }
  }

  // Determine if error repoting should be re-enabled.
  if (!should_stop_error_reporting_)
  {
    khi_msgs::msg::ErrorInfo error_info;
    driver.get_error_info(error_info.error_codes, error_info.error_msgs);
    if (error_info.error_codes.empty())
    {
      return;
    }

    // Get the current time.
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    const rclcpp::Time now = ros_clock.now();
    const auto current_time = static_cast<std::time_t>(now.seconds());
    std::tm time_info{};
    localtime_r(&current_time, &time_info);
    std::ostringstream oss;
    oss << std::put_time(&time_info, "%Y-%m-%d %H:%M:%S");
    error_info.time = oss.str();

    error_info_publisher_->publish(error_info);

    // Prevent repeated Publishing of error information.
    should_stop_error_reporting_ = true;
    old_error_codes_ = error_info.error_codes;
  }
}

/**
 * @brief Publish the actual current value.
 * @param driver
 */
void KhiPublisher::report_actual_current(const KhiDriver & driver)
{
  khi_msgs::msg::ActualCurrent msg;
  bool success = driver.get_actual_current(msg.actual_current);
  if (!success)
  {
    return;
  }
  if (msg.actual_current.empty())
  {
    return;
  }

  actual_current_publisher_->publish(msg);
}

/**
 * @brief Publish when an event occures.
 * @param driver
 * @private
 */
void KhiPublisher::publish_on_event(const KhiDriver & driver)
{
  const auto fault_qos = rclcpp::QoS(rclcpp::KeepLast(MAX_FAULTS)).reliable();
  error_info_publisher_ =
    node_->create_publisher<khi_msgs::msg::ErrorInfo>("~/error_info", fault_qos);

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiEventTriggeredPublisher Start");

  while (!should_stop_publisher_)
  {
    report_error(driver);
    std::this_thread::sleep_for(std::chrono::microseconds(EVENT_CHECK_CYCLE));
  }

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiEventTriggeredPublisher STOP");
}

/**
 * @brief Publish periodically
 * @param driver
 * @private
 */
void KhiPublisher::publish_on_schedule(const KhiDriver & driver)
{
  const auto fault_qos = rclcpp::QoS(rclcpp::KeepLast(MAX_FAULTS)).reliable();
  actual_current_publisher_ =
    node_->create_publisher<khi_msgs::msg::ActualCurrent>("~/actual_current", fault_qos);

  auto report = [&]() { report_actual_current(driver); };
  timer_ = node_->create_wall_timer(
    std::chrono::microseconds(static_cast<int64_t>(driver.get_robot().period) * 1000), report);

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiScheduledPublisher Start");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  std::thread spin_thread([&executor]() { executor.spin(); });
  while (!should_stop_publisher_)
  {
  }
  executor.cancel();

  // Wait until the thread has completely finished.
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiScheduledPublisher End");
}

}  // namespace khi_hardware
