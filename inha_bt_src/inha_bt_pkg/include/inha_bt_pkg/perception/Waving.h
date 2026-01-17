#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <atomic>
#include <mutex>
#include <string>

namespace Waving
{

class Waving : public BT::StatefulActionNode
{
public:
  Waving(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_states_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

  // State
  std::atomic_bool active_{false};
  std::atomic_bool detected_{false};
  std::atomic_bool pose_ready_{false};

  std::mutex mtx_;
  geometry_msgs::msg::PoseStamped last_goal_;

  // spam 방지용
  bool printed_waiting_{false};
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace Waving
