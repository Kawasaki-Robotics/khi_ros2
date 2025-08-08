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

#ifndef KHI_HARDWARE__KHI_PERIODIC_DATA_CONFIG_HPP_
#define KHI_HARDWARE__KHI_PERIODIC_DATA_CONFIG_HPP_

namespace khi_hardware
{
/**
 * @brief Configuration of information types that the robot periodically retrieves.
 */
struct KhiPeriodicDataConfig
{
  bool is_actual_current_enabled;
  bool is_command_current_enabled;
  bool is_actual_encorder_enabled;
  bool is_command_encorder_enabled;
  bool is_tcp_info_enabled;
  bool is_external_signal_enabled;
  bool is_internal_signal_enabled;
  bool is_ft_sensor_enabled;
};
}  // namespace khi_hardware
#endif  // KHI_HARDWARE__KHI_PERIODIC_DATA_CONFIG_HPP_
