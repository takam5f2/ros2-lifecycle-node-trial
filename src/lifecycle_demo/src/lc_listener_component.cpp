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

#include "lifecycle_demo/lc_listener.hpp"

void LCListener::data_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
}

void LCListener::notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "notify callback: Transition from state %s to %s",
  msg->start_state.label.c_str(), msg->goal_state.label.c_str());
}
