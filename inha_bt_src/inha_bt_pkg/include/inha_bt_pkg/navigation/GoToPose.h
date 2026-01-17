#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <atomic>
#include <mutex>
#include <string>

namespace GoToPose
{

class GoToPose : public BT::StatefulActionNode
{
public:
  GoToPose(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::atomic_bool started_log_once_{false};
  std::atomic_bool running_wait_log_once_{false};

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void resetState();
  bool ensureClient();
  void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose);
  void cancelGoal();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  std::string action_name_{"/navigate_to_pose"};

  std::mutex mtx_;
  typename GoalHandleNav::SharedPtr goal_handle_;

  std::atomic_bool goal_sent_{false};
  std::atomic_bool done_{false};
  std::atomic_bool success_{false};
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace GoToPose
