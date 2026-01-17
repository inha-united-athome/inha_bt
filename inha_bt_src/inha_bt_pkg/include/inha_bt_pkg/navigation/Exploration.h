#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>

namespace Exploration
{

class ExplorationAction : public BT::StatefulActionNode
{
public:
  ExplorationAction(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_resume_;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};


class ExplorationSaveGoal : public BT::SyncActionNode
{
public:
  ExplorationSaveGoal(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_save_;
};


class ExploreTrigger : public BT::SyncActionNode
{
public:
  ExploreTrigger(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_init_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_return_;
};


// GoTo처럼 메인에서 호출할 등록 함수
void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace Exploration
