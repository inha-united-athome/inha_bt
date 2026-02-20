#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <inha_interfaces/srv/set_enable.hpp>  // ✅ 너희 커스텀 srv

#include <memory>
#include <string>

namespace SetVisionDisable
{

class SetVisionDisable : public BT::SyncActionNode
{
public:
  SetVisionDisable(const std::string& name, const BT::NodeConfig& config);

  // ✅ 포트 없음 (인자 X)
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  using SrvT = inha_interfaces::srv::SetEnable;

  void ensureClient();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SrvT>::SharedPtr client_;

  static constexpr const char* kSrvName = "vision/set_enable";
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace SetVisionDisable
