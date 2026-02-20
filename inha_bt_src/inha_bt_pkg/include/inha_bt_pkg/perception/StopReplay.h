#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <inha_interfaces/srv/stop_replay.hpp>

#include <chrono>
#include <string>

namespace StopReplay
{

class StopReplay : public BT::SyncActionNode
{
public:
  using Srv = inha_interfaces::srv::StopReplay;

  StopReplay(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {};  // ✅ InputPort 없음
  }

  BT::NodeStatus tick() override;

private:
  void ensureClient();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Srv>::SharedPtr client_;

  std::string last_error_;

  static constexpr const char* kServiceName = "/grounded_sam/stop_replay";
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace StopReplay
