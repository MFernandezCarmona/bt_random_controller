// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <iostream>

#include <ctime>
#include "bt_random_controller/Turn.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_random_controller
{

using namespace std::chrono_literals;

Turn::Turn(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  std::srand(std::time(nullptr)); // use current time as seed for random generator
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);
}

void
Turn::halt()
{
}

BT::NodeStatus
Turn::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = 0.5;
  vel_pub_->publish(vel_msgs);

  auto elapsed = node_->now() - start_time_;


  double random_wait_time = 3.0 * std::rand()/RAND_MAX;
  
  if (elapsed < rclcpp::Duration::from_seconds(random_wait_time)) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace bt_random_controller

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_random_controller::Turn>("Turn");
}
