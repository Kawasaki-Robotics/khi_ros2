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

#ifndef KHI_HARDWARE__KHI_PUBLISHER_HPP_
#define KHI_HARDWARE__KHI_PUBLISHER_HPP_

#include <memory>
#include <vector>

#include "khi_msgs/msg/actual_current.hpp"
#include "khi_msgs/msg/error_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
class KhiDriver;

class KhiPublisher
{
public:
  KhiPublisher() = default;
  ~KhiPublisher();
  void start(const KhiDriver & driver);

private:
  void report_error(const KhiDriver & driver);
  void report_actual_current(const KhiDriver & driver);
  void publish_on_event(const KhiDriver & driver);
  void publish_on_schedule(const KhiDriver & driver);

  static constexpr int MAX_FAULTS = 10;
  static constexpr int EVENT_CHECK_CYCLE = 10000;  // [microsecond]

  bool should_stop_error_reporting_ = false;
  bool should_stop_publisher_ = false;
  std::vector<int> old_error_codes_{};
  std::thread event_thread_;
  std::thread schedule_thread_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<khi_msgs::msg::ErrorInfo>::SharedPtr error_info_publisher_{};
  rclcpp::Publisher<khi_msgs::msg::ActualCurrent>::SharedPtr actual_current_publisher_{};
};

}  // namespace khi_hardware

#endif  // KHI_HARDWARE__KHI_PUBLISHER_HPP_
