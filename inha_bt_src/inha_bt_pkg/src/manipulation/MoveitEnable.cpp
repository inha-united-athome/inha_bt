#include "inha_bt_pkg/manipulation/MoveitEnable.h"

using namespace std::chrono_literals;

namespace MoveitEnable
{

MoveitEnable::MoveitEnable(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("MoveitEnable: missing 'node' in blackboard");
  }
}

void MoveitEnable::resetState()
{
  started_log_once_.store(false);
  waiting_log_once_.store(false);

  req_sent_.store(false);
  done_.store(false);
  ok_.store(false);

  last_msg_.clear();
  arm_id_.clear();

  std::lock_guard<std::mutex> lk(mtx_);
  future_ = Client::SharedFuture();
}

void MoveitEnable::ensureClientOrThrow()
{
  if (!client_) {
    client_ = node_->create_client<SrvT>(kSrvName);
  }
  if (!client_->wait_for_service(kWaitService)) {
    throw BT::RuntimeError(std::string("MoveitEnable: service not available: ") + kSrvName);
  }
}

void MoveitEnable::sendRequest(const std::string& arm_id)
{
  ensureClientOrThrow();

  auto req = std::make_shared<SrvT::Request>();
  req->arm_id = arm_id;

  // Jazzy: async_send_request returns FutureAndRequestId (implicit conv deprecated)
  auto far = client_->async_send_request(req);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    future_ = far.future.share();
  }

  req_sent_.store(true);
}

BT::NodeStatus MoveitEnable::onStart()
{
  resetState();
  start_tp_ = std::chrono::steady_clock::now();

  auto arm_id_opt = getInput<std::string>("arm_id");
  if (!arm_id_opt || arm_id_opt.value().empty()) {
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "arm_id", "missing arm_id");
    return BT::NodeStatus::FAILURE;
  }
  arm_id_ = arm_id_opt.value();

  try {
    sendRequest(arm_id_);

    if (!started_log_once_.exchange(true)) {
      BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING", "request",
                    std::string("srv=") + kSrvName + " arm_id=" + arm_id_);
    }

    return BT::NodeStatus::RUNNING;

  } catch (const std::exception& e) {
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "exception",
                  std::string("srv=") + kSrvName + " arm_id=" + arm_id_ + " err=" + e.what());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveitEnable::onRunning()
{
  if (done_.load()) {
    // 결과가 이미 확정된 상태
    if (ok_.load()) {
      BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS", "response",
                    std::string("srv=") + kSrvName + " arm_id=" + arm_id_ + " msg=" + last_msg_);
      return BT::NodeStatus::SUCCESS;
    } else {
      BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "response",
                    std::string("srv=") + kSrvName + " arm_id=" + arm_id_ + " msg=" + last_msg_);
      return BT::NodeStatus::FAILURE;
    }
  }

  // timeout 체크
  const auto now = std::chrono::steady_clock::now();
  if (now - start_tp_ > kTimeout) {
    last_msg_ = "timeout";
    done_.store(true);
    ok_.store(false);
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "response",
                  std::string("timeout srv=") + kSrvName + " arm_id=" + arm_id_);
    return BT::NodeStatus::FAILURE;
  }

  Client::SharedFuture fut;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    fut = future_;
  }

  if (!req_sent_.load() || !fut.valid()) {
    if (!waiting_log_once_.exchange(true)) {
      BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING", "wait(future)",
                    std::string("srv=") + kSrvName + " arm_id=" + arm_id_);
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto st = fut.wait_for(0ms);
  if (st != std::future_status::ready) {
    if (!waiting_log_once_.exchange(true)) {
      BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING", "wait(response)",
                    std::string("srv=") + kSrvName + " arm_id=" + arm_id_);
    }
    return BT::NodeStatus::RUNNING;
  }

  try {
    auto resp = fut.get();
    done_.store(true);

    ok_.store(resp->success);
    last_msg_ = resp->message;

    if (resp->success) {
      BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS", "response",
                    std::string("srv=") + kSrvName + " arm_id=" + arm_id_ + " msg=" + resp->message);
      return BT::NodeStatus::SUCCESS;
    } else {
      BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "response",
                    std::string("srv=") + kSrvName + " arm_id=" + arm_id_ + " msg=" + resp->message);
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    done_.store(true);
    ok_.store(false);
    last_msg_ = e.what();
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "exception",
                  std::string("srv=") + kSrvName + " arm_id=" + arm_id_ + " err=" + e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void MoveitEnable::onHalted()
{
  // 서비스는 cancel 개념이 없으므로 상태만 초기화
  BT_LOG_STATUS(node_->get_logger(), name(), "HALTED", "noop",
                std::string("srv=") + kSrvName + " arm_id=" + arm_id_);
  resetState();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<MoveitEnable>("MoveitEnable");
}

} // namespace MoveitEnable
