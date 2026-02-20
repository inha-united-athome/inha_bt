// WavingApproach.h
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <atomic>
#include <mutex>
#include <string>

#include <inha_interfaces/action/waving_approach.hpp>

namespace WavingApproach
{

class WavingApproach : public BT::StatefulActionNode
{
public:
  WavingApproach(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Final pose returned by action")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using ApproachAction = inha_interfaces::action::WavingApproach;
  using GoalHandleApproach = rclcpp_action::ClientGoalHandle<ApproachAction>;

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ApproachAction>::SharedPtr client_;

  // State
  std::atomic_bool done_{false};
  std::atomic_bool ok_{false};

  std::mutex mtx_;
  GoalHandleApproach::SharedPtr goal_handle_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  bool printed_waiting_{false};

  void ensureClient();

  static constexpr const char* kActionName = "perception/waving_approach";
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace WavingApproach
