#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <string>
#include <behaviortree_cpp/bt_factory.h>

// 너희 실제 action 헤더로 교체
#include <inha_interfaces/action/visual_align.hpp>

namespace VisualAlign
{

class VisualAlign : public BT::StatefulActionNode
{
public:
  using VisualAlignAction = inha_interfaces::action::VisualAlign;
  using GoalHandleVA = rclcpp_action::ClientGoalHandle<VisualAlignAction>;

  VisualAlign(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("enable", true, "goal.enable"),
      BT::OutputPort<std::string>("error_message", "result.error_message"),
    };
  }

private:
  void ensureClient();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<VisualAlignAction>::SharedPtr client_;

  static constexpr const char* kActionName = "perception/visual_align";

  // 최소 상태
  std::atomic_bool done_{false};
  std::atomic_bool ok_{false};
  std::atomic_bool printed_waiting_{false};

  std::string last_error_;

  // halt 시 cancel용 (선택적으로만 유지)
  GoalHandleVA::SharedPtr goal_handle_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace VisualAlign
