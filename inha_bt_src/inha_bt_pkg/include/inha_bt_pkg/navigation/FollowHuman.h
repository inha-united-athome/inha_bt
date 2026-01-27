#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "inha_interfaces/action/follow_human.hpp"

namespace FollowHuman
{

class FollowHumanBT : public BT::StatefulActionNode
{
public:
  using FollowHumanAction = inha_interfaces::action::FollowHuman;
  using GoalHandleT = rclcpp_action::ClientGoalHandle<FollowHumanAction>;
  using ClientT = rclcpp_action::Client<FollowHumanAction>;

  FollowHumanBT(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;

  std::shared_ptr<ClientT> client_;
  GoalHandleT::SharedPtr goal_handle_;

  std::shared_future<GoalHandleT::SharedPtr> goal_future_;
  std::shared_future<typename GoalHandleT::WrappedResult> result_future_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace FollowHuman
