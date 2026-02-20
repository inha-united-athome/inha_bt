#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <inha_interfaces/srv/set_enable.hpp>

#include <string>
#include <unordered_map>
#include <mutex>

namespace ServiceTrigger
{

class ServiceTrigger : public BT::SyncActionNode
{
public:
  ServiceTrigger(const std::string& name, const BT::NodeConfig& config);

  // ✅ service_name만 받는 공용 트리거
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "service_name",
        "/manipulation/graspgen/enable",
        "target SetEnable service name (sends {enable: true})"
      ),
      BT::InputPort<bool>(
        "enable",
        true,
        "value to send as Request.enable"
      )
    };
  }

  BT::NodeStatus tick() override;

private:
  using SrvT = inha_interfaces::srv::SetEnable;

  rclcpp::Node::SharedPtr node_;

  // ✅ 여러 서비스명에 대해 client 재사용 (성능/안정)
  std::mutex mu_;
  std::unordered_map<std::string, rclcpp::Client<SrvT>::SharedPtr> clients_;

  rclcpp::Client<SrvT>::SharedPtr getClientLocked(const std::string& srv_name);
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace ServiceTrigger
