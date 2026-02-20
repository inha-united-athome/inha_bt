#include "inha_bt_pkg/perception/StartReplay.h"

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

namespace StartReplay
{

StartReplay::StartReplay(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("StartReplay: missing 'node' in blackboard");
  }
  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "got node from blackboard");
}

void StartReplay::ensureClient()
{
  if (client_) return;

  client_ = node_->create_client<Srv>(kServiceName);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "READY", "out(srv)",
                std::string("client created: ") + kServiceName);
}

BT::NodeStatus StartReplay::tick()
{
  ensureClient();
  last_error_.clear();

  // ✅ InputPort 2개만 사용
  double rate = 1.0;
  bool publish_bg = true;
  (void)getInput("rate", rate);
  (void)getInput("publish_background", publish_bg);

  BT_LOG_STATUS(
    node_->get_logger(), this->name(), "START", "out(srv)",
    std::string("call StartReplay(rate=") + std::to_string(rate) +
      ", publish_background=" + (publish_bg ? "true" : "false") + ")"
  );

  if (!client_->wait_for_service(300ms)) {
    last_error_ = std::string("service not available: ") + kServiceName;
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(srv)", last_error_);
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<Srv::Request>();
  req->rate = rate;
  req->publish_background = publish_bg;

  auto future = client_->async_send_request(req);

  BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(resp)");

  // ✅ spin_until_future_complete 금지 (bt_main에서 이미 exec.spin 중)
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
  factory.registerNodeType<StartReplay>("StartReplay");
}

}  // namespace StartReplay
