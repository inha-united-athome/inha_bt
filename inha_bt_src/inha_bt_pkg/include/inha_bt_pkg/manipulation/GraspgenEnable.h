#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <mutex>
#include <string>

// ⚠️ 너희 srv 실제 파일명/경로에 맞게 수정
#include <inha_interfaces/srv/graspgen_enable.hpp>

namespace GraspgenEnable
{

class GraspgenEnable : public BT::StatefulActionNode
{
public:
  GraspgenEnable(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using Srv = inha_interfaces::srv::GraspgenEnable;
  using Client = rclcpp::Client<Srv>;

  void resetState();
  bool ensureClient();
  void sendRequest(const std::string& arm_id);

  rclcpp::Node::SharedPtr node_;
  Client::SharedPtr client_;

  std::string service_name_{"/manipulation/graspgen/enable"};

  std::mutex mtx_;
  Client::SharedFuture future_;

  std::atomic_bool started_log_once_{false};
  std::atomic_bool running_wait_log_once_{false};

  std::atomic_bool req_sent_{false};
  std::atomic_bool done_{false};
  std::atomic_bool success_{false};
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace GraspgenEnable
