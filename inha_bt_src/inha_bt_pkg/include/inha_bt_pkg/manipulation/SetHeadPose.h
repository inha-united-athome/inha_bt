#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <inha_interfaces/action/set_head_pose.hpp>

namespace SetHeadPose
{

class SetHeadPoseBT : public BT::StatefulActionNode
{
public:
  using ActionT = inha_interfaces::action::SetHeadPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  SetHeadPoseBT(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("head_0", 0.0, "pan (rad)"),
      BT::InputPort<double>("head_1", 0.0, "tilt (rad)"),
      BT::InputPort<double>("duration", 1.0, "motion duration (sec)"),
      BT::InputPort<std::string>("action_name", "/rby1/set_head_pose", "head pose action server name"),
      BT::OutputPort<std::string>("error_message", "")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;

  std::string action_name_;

  std::mutex mtx_;
  GoalHandle::SharedPtr goal_handle_;

  std::atomic_bool done_{false};
  std::atomic_bool ok_{false};
  std::string error_message_;

  void resetState();
  bool ensureClient();
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace SetHeadPose
