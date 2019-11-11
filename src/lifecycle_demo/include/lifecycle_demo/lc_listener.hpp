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

#ifndef __LC_LISTENER_HPP__
#define __LC_LISTENER_HPP__

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"


/// LC_Listener class as a simple listener node
/**
 * We subscribe to two topics
 * - /@namespace/lifecycle_chatter: The data topic from the talker
 * - /@namespace/lc_talker__transition_event: The topic publishing
 *   notifications about state changes of the node
 *   lc_talker
 */
class LCListener : public rclcpp::Node
{
public:
  explicit LCListener(const std::string & node_name)
  : Node(node_name)
  {
    // Data topic from the lc_talker node
    std::string this_namespace = this->get_namespace();
    std::string chatter_topic_name = "lc_chatter";
    std::string trans_topic_name = "/lc_talker/transition_event";
    if (this_namespace != "/") {
      chatter_topic_name = this_namespace + "lc_chatter";
      trans_topic_name = this_namespace + trans_topic_name;
    }

    // Data topic receiving from talker node.
    sub_data_ = this->create_subscription<std_msgs::msg::String>(
      chatter_topic_name.c_str(),
      10,
      std::bind(&LCListener::data_callback, this, std::placeholders::_1));

    // Notification event topic.
    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      trans_topic_name.c_str(),
      10,
      std::bind(&LCListener::notification_callback, this, std::placeholders::_1)
    );
  }

  void data_callback(const std_msgs::msg::String::SharedPtr msg);
  void notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>>
  sub_notification_;

};

#endif