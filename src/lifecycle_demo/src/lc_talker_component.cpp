// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_demo/lc_talker.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;


void LCTalker::send_msg(void) {
  static size_t count = 0;
  State current_state = get_current_state();

  if (current_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCUTILS_LOG_INFO_NAMED(node_fullname_.c_str(), "I'm not active");
    return;
  }
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Sending message #" + std::to_string(++count) 
    + " from " + node_fullname_;

  pub_->publish(std::move(msg));
  
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;

CallbackReturn LCTalker::on_configure(const State &) {
  std::string this_namespace = get_namespace();
  std::string chatter_topic_name = "lc_chatter";
  RCUTILS_LOG_INFO_NAMED(node_fullname_.c_str(), "node_name: %s", this_namespace.c_str());

  if (this_namespace != "/") {
    this->node_fullname_ = this_namespace + std::string("/") + std::string(get_name());
    chatter_topic_name = this_namespace + "/lc_chatter";
  } else
    this->node_fullname_ = std::string(get_name());

  pub_ = this->create_publisher<std_msgs::msg::String>(
    chatter_topic_name.c_str(),
    10
  );

  timer_ = this->create_wall_timer(
    1s, std::bind(&LCTalker::send_msg, this));

  RCUTILS_LOG_INFO_NAMED(node_fullname_.c_str(), "on_configure() is called.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn LCTalker::on_activate(const State &) {
  pub_->on_activate();

  RCUTILS_LOG_INFO_NAMED(node_fullname_.c_str(), "on_activate() is called.");

  std::this_thread::sleep_for(2s);
  return CallbackReturn::SUCCESS;
}

CallbackReturn LCTalker::on_deactivate(const State &) {
  pub_->on_deactivate();

  RCUTILS_LOG_INFO_NAMED(node_fullname_.c_str(), "on_deactivate() is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LCTalker::on_cleanup(const State &) {
  pub_.reset();
  timer_.reset();

  RCUTILS_LOG_INFO_NAMED(node_fullname_.c_str(), "on_cleanup() is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LCTalker::on_shutdown(const State & state) {
  pub_.reset();
  timer_.reset();

  RCUTILS_LOG_INFO_NAMED(
    node_fullname_.c_str(),
    "on_shutdown() is called from state %s.",
    state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn LCTalker::on_error(const State & state) {
  pub_.reset();
  timer_.reset();
  RCUTILS_LOG_INFO_NAMED(
    node_fullname_.c_str(),
    "on_error() is called due to error caused in state %s",
    state.label().c_str());
  return CallbackReturn::SUCCESS; 
}
