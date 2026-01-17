#pragma once

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <inha_interfaces/action/vlm.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include <atomic>
#include <mutex>
#include <string>
#include <chrono>

namespace ExtractWordBT
{

class ExtractWord : public BT::StatefulActionNode
{
public:
  using VLM = inha_interfaces::action::Vlm;
  using GoalHandleVLM = rclcpp_action::ClientGoalHandle<VLM>;

  ExtractWord(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<VLM>::SharedPtr client_;
  GoalHandleVLM::SharedPtr goal_handle_;

  // state
  std::mutex mx_;
  std::atomic<bool> finished_{false};
  bool success_{false};

  // inputs
  std::string heard_text_;
  std::string prompt_;
  std::string pick_;
  int word_count_{1};
  int vlm_mode_{45};
  std::string action_name_{"/vlm/query"};
  int timeout_ms_{8000};

  // result
  std::string generated_text_;
  std::string result_json_;
  std::string last_error_;

  // timing
  std::chrono::steady_clock::time_point start_tp_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace ExtractWordBT
