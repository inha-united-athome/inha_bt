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

namespace GoToPrecise
{

class GoToPrecise : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ClientNav      = rclcpp_action::Client<NavigateToPose>;

  GoToPrecise(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void resetState();
  bool ensureClient();
  void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose);
  void cancelGoal();

  // error helper (thread-safe)
  void setLastError(const std::string& msg);
  std::string getLastErrorCopy() const;

  rclcpp::Node::SharedPtr node_;
  typename ClientNav::SharedPtr client_;

  // ports (override 가능)
  std::string action_name_{"/navigate_to_pose"};
  std::string behavior_tree_{"goal_precise.xml"};

  // shared state
  mutable std::mutex mtx_;
  GoalHandleNav::SharedPtr goal_handle_;
  std::string last_error_;

  std::atomic_bool started_log_once_{false};
  std::atomic_bool running_wait_log_once_{false};

  std::atomic_bool goal_sent_{false};
  std::atomic_bool done_{false};
  std::atomic_bool success_{false};
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace GoToPrecise
