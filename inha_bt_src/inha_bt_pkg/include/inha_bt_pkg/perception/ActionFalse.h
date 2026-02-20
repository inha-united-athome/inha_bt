// ===== include/inha_bt_pkg/perception/ActionFalse.h =====
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <mutex>
#include <string>

#include <inha_interfaces/action/waving.hpp>

namespace ActionFalse
{

class ActionFalse : public BT::StatefulActionNode
{
public:
  ActionFalse(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("error_message", "ignored in this node (always success)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using Act = inha_interfaces::action::Waving;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Act>;

  void ensureClient();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Act>::SharedPtr client_;

  std::atomic_bool active_{false};
  std::atomic_bool done_{false};
  std::atomic_bool ok_{false};

  std::mutex mtx_;
  GoalHandle::SharedPtr goal_handle_;
  std::string last_error_;

  bool printed_waiting_{false};

  static constexpr const char* kActionName = "/perception/waving";
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace ActionFalse
