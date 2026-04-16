// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_path_side_shift_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void SideShiftModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {});

  SideShiftParameters p{};

  const std::string ns = "side_shift.";
  p.min_distance_to_start_shifting =
    node->declare_parameter<double>(ns + "min_distance_to_start_shifting");
  p.time_to_start_shifting = node->declare_parameter<double>(ns + "time_to_start_shifting");
  p.shifting_lateral_jerk = node->declare_parameter<double>(ns + "shifting_lateral_jerk");
  p.min_shifting_distance = node->declare_parameter<double>(ns + "min_shifting_distance");
  p.min_shifting_speed = node->declare_parameter<double>(ns + "min_shifting_speed");
  p.shift_request_time_limit = node->declare_parameter<double>(ns + "shift_request_time_limit");
  p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker");

  parameters_ = std::make_shared<SideShiftParameters>(p);

  // Example service: returns current side-shift offset limits.
  srv_get_offset_limits_ = node->create_service<std_srvs::srv::Trigger>(
    ns + "calc_offset_limits",
    [this](
      [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      for (const auto & observer : observers_) {
        if (observer.expired()) {
          continue;
        }
        const auto side_shift_module = std::dynamic_pointer_cast<SideShiftModule>(observer.lock());
        if (!side_shift_module) {
          continue;
        }
        const auto [min_offset, max_offset] = side_shift_module->getOffsetLimits();
        std::ostringstream message;
        message << "offset_limits: min=" << min_offset << ", max=" << max_offset;
        response->success = true;
        response->message = message.str();
        return;
      }

      if (idle_module_ptr_) {
        const auto * idle_side_shift_module = dynamic_cast<SideShiftModule *>(idle_module_ptr_.get());
        if (idle_side_shift_module != nullptr) {
          const auto [min_offset, max_offset] = idle_side_shift_module->getOffsetLimits();
          std::ostringstream message;
          message << "offset_limits(from_idle): min=" << min_offset << ", max=" << max_offset;
          response->success = true;
          response->message = message.str();
          return;
        }
      }

      response->success = false;
      response->message = "No active side shift module instance.";
    });
}

void SideShiftModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  [[maybe_unused]] auto p = parameters_;

  [[maybe_unused]] const std::string ns = "side_shift.";
  // update_param<bool>(parameters, ns + ..., ...);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SideShiftModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
