#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>

#include <mutex>
#include <algorithm>
#include <cstdint>

namespace WaitGoalReached
{

class WaitGoalReached : public BT::StatefulActionNode
{
public:
  WaitGoalReached(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();

private:
  void onStatus_(action_msgs::msg::GoalStatusArray::SharedPtr msg);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int8_t last_status_{-1};
  bool printed_wait_{false};
  std::string fail_reason_;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_status_;

  std::mutex m_;
  bool active_{false};
  bool done_{false};
  bool success_{false};

  double timeout_sec_{60.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  bool target_set_{false};
  int64_t target_stamp_ns_{0};
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace WaitGoalReached
