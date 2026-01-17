#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

namespace ContextJsonMaker
{

class ContextJsonMaker : public BT::SyncActionNode
{
public:
  ContextJsonMaker(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // optional logger
  rclcpp::Node::SharedPtr node_;
};

// 🔹 BT Factory 등록 함수 선언
void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace ContextJsonMaker
