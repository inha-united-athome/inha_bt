#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <chrono>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <inha_interfaces/action/listen.hpp>

namespace Listen
{

class Listen : public BT::StatefulActionNode
{
public:
  Listen(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;

  using ListenAction = inha_interfaces::action::Listen;
  using GoalHandleListen = rclcpp_action::ClientGoalHandle<ListenAction>;

  rclcpp_action::Client<ListenAction>::SharedPtr client_;
  GoalHandleListen::SharedPtr goal_handle_;

  std::atomic_bool finished_{false};
  bool success_{false};

  std::string heard_text_;
  std::string last_state_;
  std::string last_error_;

  float timeout_sec_{5.0f};
  std::chrono::steady_clock::time_point start_tp_;

  std::mutex mx_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace Listen
