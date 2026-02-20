#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include <inha_interfaces/srv/execute_success.hpp>

namespace ExecuteSuccessServer
{

class ExecuteSuccessServer : public BT::StatefulActionNode
{
public:
  ExecuteSuccessServer(const std::string& name, const BT::NodeConfig& config);

  // ✅ 포트 없음: 서비스로만 결과 받음
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // TODO: 너희 실제 srv 타입으로 교체
  using SrvT = inha_interfaces::srv::ExecuteSuccess;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<SrvT>::SharedPtr srv_;

  std::atomic<bool> got_request_{false};
  std::atomic<bool> last_result_{false};

  std::mutex mu_;
  std::string last_msg_;

  static constexpr const char* kSrvName = "/manipulation/execute/success";

  void ensureServer();
};

// ----------------- RegisterNodes -----------------
void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace ExecuteSuccessServer
