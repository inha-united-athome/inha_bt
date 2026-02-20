#include "inha_bt_pkg/perception/ObjectSwitch.h"

#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace
{
// ---- 공통: 요약 로그 헬퍼 ----
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

namespace SetVisionDisable
{

SetVisionDisable::SetVisionDisable(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("SetVisionDisable: missing 'node' in blackboard");
  }
  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "got node from blackboard");
}

void SetVisionDisable::ensureClient()
{
  if (client_) return;
  client_ = node_->create_client<SrvT>(kSrvName);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "READY", "out(srv)",
                std::string("client created: ") + kSrvName);
}

BT::NodeStatus SetVisionDisable::tick()
{
  ensureClient();

  // ✅ "disable 노드"니까 항상 끄기
  // (너희 srv가 enable 필드면 enable=false)
  // (srv가 disable 필드면 disable=true로 바꾸면 됨)
  if (!client_->wait_for_service(300ms)) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(srv)",
                  std::string(kSrvName) + " not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<SrvT::Request>();
  req->enable = false;  // ✅ 너희 srv 정의에 맞게 유지/수정

  BT_LOG_STATUS(node_->get_logger(), this->name(), "START", "out(srv)", "send enable=false");

  auto fut = client_->async_send_request(req);

  // ✅ 핵심: bt_main에서 이미 executor가 spin 중이므로 여기서 spin 금지
  if (fut.wait_for(700ms) != std::future_status::ready) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(srv)", "service call timeout");
    return BT::NodeStatus::FAILURE;
  }

  auto resp = fut.get();

  if (resp->success) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "in(srv)",
                  resp->message.empty() ? "disabled" : resp->message);
    return BT::NodeStatus::SUCCESS;
  } else {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(srv)",
                  resp->message.empty() ? "success=false" : resp->message);
    return BT::NodeStatus::FAILURE;
  }
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetVisionDisable>("SetVisionDisable");
}

} // namespace SetVisionDisable
