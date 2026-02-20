#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include <inha_interfaces/srv/moveit_enable.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

namespace MoveitEnable
{

// ---- 공통: 요약 로그 헬퍼 ----
static inline void BT_LOG_STATUS(
  const rclcpp::Logger& logger,
  const std::string& node_name,
  const char* status,
  const char* io,
  const std::string& reason = "")
{
  if (std::string(status) == "SUCCESS") {
    if (reason.empty()) RCLCPP_INFO(logger, "[BT][%s] %s | %s", node_name.c_str(), status, io);
    else RCLCPP_INFO(logger, "[BT][%s] %s | %s | %s", node_name.c_str(), status, io, reason.c_str());
  } else {
    if (reason.empty()) RCLCPP_WARN(logger, "[BT][%s] %s | %s", node_name.c_str(), status, io);
    else RCLCPP_WARN(logger, "[BT][%s] %s | %s | %s", node_name.c_str(), status, io, reason.c_str());
  }
}

class MoveitEnable : public BT::StatefulActionNode
{
public:
  using SrvT   = inha_interfaces::srv::MoveitEnable;
  using Client = rclcpp::Client<SrvT>;

  MoveitEnable(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("arm_id") };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  static constexpr const char* kSrvName = "/manipulation/moveit/enable";

  void resetState();
  void ensureClientOrThrow();
  void sendRequest(const std::string& arm_id);

  rclcpp::Node::SharedPtr node_;
  Client::SharedPtr client_;

  // async state
  std::mutex mtx_;
  Client::SharedFuture future_;   // response future
  std::string arm_id_;

  std::atomic_bool req_sent_{false};
  std::atomic_bool done_{false};
  std::atomic_bool ok_{false};

  std::string last_msg_;

  // log once
  std::atomic_bool started_log_once_{false};
  std::atomic_bool waiting_log_once_{false};

  // timeout
  std::chrono::steady_clock::time_point start_tp_;
  static constexpr auto kTimeout = std::chrono::minutes(2);
  static constexpr auto kWaitService = std::chrono::seconds(5);
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace MoveitEnable
