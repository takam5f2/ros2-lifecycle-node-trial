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

#ifndef __LC_TALKER_HPP__
#define __LC_TALKER_HPP__

#include <memory>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/// LCTalker inheriting from rclcpp_lifecycle::LifecycleNode
class LCTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  // Contructor.
  explicit LCTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}
  
  // cyclic function.
  void send_msg(void);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;

  // state transition definition.
  CallbackReturn on_configure(const State &);
  CallbackReturn on_activate(const State &);
  CallbackReturn on_deactivate(const State &);
  CallbackReturn on_cleanup(const State &);
  CallbackReturn on_shutdown(const State & state);
  CallbackReturn on_error(const State & state);

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::string node_fullname_;
};


#endif