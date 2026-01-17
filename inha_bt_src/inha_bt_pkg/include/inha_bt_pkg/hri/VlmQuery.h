#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include "inha_interfaces/action/vlm.hpp"

namespace VlmQuery
{

class VlmBT : public BT::StatefulActionNode
{
public:
  using VlmAction = inha_interfaces::action::Vlm;
  using GoalHandleVlm = rclcpp_action::ClientGoalHandle<VlmAction>;

  VlmBT(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<VlmAction>::SharedPtr client_;
  GoalHandleVlm::SharedPtr goal_handle_;

  std::string action_name_;
  int timeout_ms_{8000};
  int mode_{1};
  std::string text_;
  std::string prompt_;
  std::string context_json_;

  std::mutex mx_;
  std::atomic<bool> finished_{false};
  bool success_{false};
  std::string generated_text_;
  std::string error_message_;

  std::chrono::steady_clock::time_point start_tp_;

  void ensureClient();
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace VlmQuery
