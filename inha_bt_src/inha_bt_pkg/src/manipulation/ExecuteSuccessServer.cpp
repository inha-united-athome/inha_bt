#include "inha_bt_pkg/manipulation/ExecuteSuccessServer.h"

namespace ExecuteSuccessServer
{

ExecuteSuccessServer::ExecuteSuccessServer(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("ExecuteSuccessServer: missing 'node' in blackboard");
  }
}

void ExecuteSuccessServer::ensureServer()
{
  if (srv_) return;

  srv_ = node_->create_service<SrvT>(
    kSrvName,
    [this](const std::shared_ptr<SrvT::Request> req,
           std::shared_ptr<SrvT::Response> resp)
    {
      // 요청 수신
      last_result_.store(req->result, std::memory_order_relaxed);
      got_request_.store(true, std::memory_order_release);

      {
        std::lock_guard<std::mutex> lk(mu_);
        last_msg_ = std::string("received result=") + (req->result ? "true" : "false");
      }

      // 응답
      resp->received = true;
      resp->message = "ok";

      RCLCPP_INFO(node_->get_logger(),
                  "[ExecuteSuccessServer] got request: result=%s",
                  req->result ? "true" : "false");
    }
  );

  RCLCPP_INFO(node_->get_logger(),
              "[ExecuteSuccessServer] service server ready: %s",
              kSrvName);
}

BT::NodeStatus ExecuteSuccessServer::onStart()
{
  ensureServer();

  // 새 실행 시작할 때는 플래그 초기화
  got_request_.store(false, std::memory_order_release);
  last_result_.store(false, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lk(mu_);
    last_msg_.clear();
  }

  RCLCPP_INFO(node_->get_logger(), "[ExecuteSuccessServer] waiting for /execute/success...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteSuccessServer::onRunning()
{
  // 서비스 요청 들어올 때까지 RUNNING
  if (!got_request_.load(std::memory_order_acquire)) {
    return BT::NodeStatus::RUNNING;
  }

  const bool res = last_result_.load(std::memory_order_relaxed);

  std::string msg;
  {
    std::lock_guard<std::mutex> lk(mu_);
    msg = last_msg_;
  }

  if (res) {
    RCLCPP_INFO(node_->get_logger(), "[ExecuteSuccessServer] BT SUCCESS (%s)", msg.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "[ExecuteSuccessServer] BT FAILURE (%s)", msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

void ExecuteSuccessServer::onHalted()
{
  // 중단되면 다음 실행을 위해 플래그만 정리
  got_request_.store(false, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lk(mu_);
    last_msg_.clear();
  }

  RCLCPP_WARN(node_->get_logger(), "[ExecuteSuccessServer] halted");
}

// ----------------- RegisterNodes -----------------
void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ExecuteSuccessServer>("ExecuteSuccessServer");
}

} // namespace ExecuteSuccessServer
