#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "inha_interfaces/action/set_robot_pose.hpp"

namespace SetRobotPose
{

class SetRobotPoseBT : public BT::StatefulActionNode
{
public:
  using ActionT     = inha_interfaces::action::SetRobotPose;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<ActionT>;

  SetRobotPoseBT(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;

  std::mutex mx_;
  std::atomic_bool done_{false};
  std::atomic_bool ok_{false};

  typename GoalHandle::SharedPtr goal_handle_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace SetRobotPose
