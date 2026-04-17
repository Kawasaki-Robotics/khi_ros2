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
KhiPublisher::~KhiPublisher() { stop(); }

void KhiPublisher::init(const KhiDriver & driver)
{
  std::string node_namespace = "/khi_controller" + std::to_string(driver.get_robot().controller_no);
  node_ = rclcpp::Node::make_shared("khi_publisher", node_namespace);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
}

void KhiPublisher::start(const KhiDriver & driver)
{
  should_stop_publisher_.store(false);
  should_stop_error_reporting_ = false;

  if (!executor_spin_thread_.joinable())
  {
    start_executor();
  }

  const auto fault_qos = rclcpp::QoS(rclcpp::KeepLast(MAX_FAULTS)).reliable();
  error_info_publisher_ =
    node_->create_publisher<khi_msgs::msg::ErrorInfo>("~/error_info", fault_qos);
  actual_current_publisher_ =
    node_->create_publisher<khi_msgs::msg::ActualCurrent>("~/actual_current", fault_qos);

  auto report = [this, &driver]() { report_actual_current(driver); };
  timer_ = node_->create_wall_timer(
    std::chrono::microseconds(static_cast<int64_t>(driver.get_robot().period) * 1000), report);

  auto event = [this, &driver]() { publish_on_event(driver); };
  event_thread_ = std::thread(event);

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiPublisher Start");
}

void KhiPublisher::stop()
{
  should_stop_publisher_.store(true);
  if (timer_)
  {
    timer_->cancel();
  }
  if (event_thread_.joinable())
  {
    event_thread_.join();
  }
  stop_executor();
  timer_.reset();
  error_info_publisher_.reset();
  actual_current_publisher_.reset();
}

void KhiPublisher::start_executor()
{
  exit_.store(false);
  executor_spin_thread_ = std::thread(
    [this]()
    {
      while (!exit_.load())
      {
        executor_->spin_once(std::chrono::milliseconds(100));
      }
    });
}

void KhiPublisher::stop_executor()
{
  exit_.store(true);
  if (executor_)
  {
    executor_->cancel();
  }
  if (executor_spin_thread_.joinable())
  {
    executor_spin_thread_.join();
  }
}

void KhiPublisher::report_error(const KhiDriver & driver)
{
  std::vector<int> error_codes;
  std::vector<std::string> error_msgs;
  driver.get_error_info(error_codes, error_msgs);

  // Determine if error repoting should be re-enabled.
  if (should_stop_error_reporting_ && (error_codes != old_error_codes_))
  {
    should_stop_error_reporting_ = false;
  }
  old_error_codes_ = error_codes;

  // Report error if error reporting is enabled.
  if (!should_stop_error_reporting_)
  {
    if (!error_codes.empty())
    {
      khi_msgs::msg::ErrorInfo error_info;

      // Get the current time.
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      const rclcpp::Time now = ros_clock.now();

      // Keep the string-based time for backward compatibility.
      const auto current_time = static_cast<std::time_t>(now.seconds());
      std::tm time_info{};
      localtime_r(&current_time, &time_info);
      std::ostringstream oss;
      oss << std::put_time(&time_info, "%Y-%m-%d %H:%M:%S");
      error_info.time = oss.str();

      error_info.stamp = now;
      error_info.error_codes = error_codes;
      error_info.error_msgs = error_msgs;

      error_info_publisher_->publish(error_info);
    }

    // Prevent repeated Publishing of error information.
    should_stop_error_reporting_ = true;
  }
}

/**
 * @brief Publish the actual current value.
 * @param driver
 */
void KhiPublisher::report_actual_current(const KhiDriver & driver)
{
  // Guard: return early if stop() is in progress to avoid using reset publishers.
  if (should_stop_publisher_.load())
  {
    return;
  }

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
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiEventTriggeredPublisher Start");

  while (!should_stop_publisher_.load())
  {
    report_error(driver);
    std::this_thread::sleep_for(std::chrono::microseconds(EVENT_CHECK_CYCLE));
  }

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiEventTriggeredPublisher STOP");
}

}  // namespace khi_hardware
