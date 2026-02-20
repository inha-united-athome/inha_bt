#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "inha_interfaces/srv/start_replay.hpp"

namespace StartReplay
{

class StartReplay : public BT::SyncActionNode
{
public:
  using Srv = inha_interfaces::srv::StartReplay;

  StartReplay(const std::string& name, const BT::NodeConfig& config);

  // ✅ InputPort 2개만
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("rate", 1.0, "replay rate (e.g., 1.0)"),
      BT::InputPort<bool>("publish_background", true, "publish background outputs")
    };
  }

  BT::NodeStatus tick() override;

private:
  void ensureClient();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Srv>::SharedPtr client_;
  std::string last_error_;

  static constexpr const char* kServiceName = "/grounded_sam/start_replay";  // ✅ 실제 서비스 이름
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace StartReplay
