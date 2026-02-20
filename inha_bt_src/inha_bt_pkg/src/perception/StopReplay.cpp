#include "inha_bt_pkg/perception/StopReplay.h"

using namespace std::chrono_literals;

namespace
{
static inline void BT_LOG_STATUS(
  const rclcpp::Logger& logger,
  const std::string& node_name,
  const char* status,
  const char* io,
  const std::string& reason = "")
{
  if (!reason.empty()) {
    RCLCPP_WARN(logger, "[BT] %s : %s  (%s)  reason=%s",
                node_name.c_str(), status, io, reason.c_str());
  } else {
    RCLCPP_INFO(logger, "[BT] %s : %s  (%s)",
                node_name.c_str(), status, io);
  }
}
} // namespace

namespace StopReplay
{

StopReplay::StopReplay(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("StopReplay: missing 'node' in blackboard");
  }
  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "got node from blackboard");
}

void StopReplay::ensureClient()
{
  if (client_) return;

  client_ = node_->create_client<Srv>(kServiceName);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "READY", "out(srv)",
                std::string("client created: ") + kServiceName);
}

BT::NodeStatus StopReplay::tick()
{
  ensureClient();
  last_error_.clear();

  BT_LOG_STATUS(node_->get_logger(), this->name(), "START", "out(srv)", "call StopReplay()");

  if (!client_->wait_for_service(300ms)) {
    last_error_ = std::string("service not available: ") + kServiceName;
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(srv)", last_error_);
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<Srv::Request>();  // 빈 request
  auto future = client_->async_send_request(req);

  BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(resp)");

  if (future.wait_for(2s) != std::future_status::ready) {
    last_error_ = "service call timeout/fail";
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(resp)", last_error_);
    return BT::NodeStatus::FAILURE;
  }

  const auto resp = future.get();
  if (resp->success) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "in(resp)", resp->message);
    return BT::NodeStatus::SUCCESS;
  }

  last_error_ = resp->message.empty() ? "success=false" : resp->message;
  BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(resp)", last_error_);
  return BT::NodeStatus::FAILURE;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<StopReplay>("StopReplay");
}

}  // namespace StopReplay
