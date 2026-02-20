#include "inha_bt_pkg/manipulation/GraspgenEnable.h"

#include <chrono>

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

namespace GraspgenEnable
{

GraspgenEnable::GraspgenEnable(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("GraspgenEnable: missing 'node' in blackboard");
  }
  ensureClient();
}

BT::PortsList GraspgenEnable::providedPorts()
{
  return {
    BT::InputPort<std::string>("arm_id", "target arm id (e.g., left/right)"),
    BT::InputPort<std::string>(
      "service_name",
      std::string("/manipulation/graspgen/enable"),
      "service name override"),
  };
}

void GraspgenEnable::resetState()
{
  started_log_once_.store(false);
  running_wait_log_once_.store(false);

  req_sent_.store(false);
  done_.store(false);
  success_.store(false);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    future_ = Client::SharedFuture();
  }
}

bool GraspgenEnable::ensureClient()
{
  if (!node_) return false;
  if (!client_) {
    client_ = node_->create_client<Srv>(service_name_);
  }
  return (client_ != nullptr);
}

void GraspgenEnable::sendRequest(const std::string& arm_id)
{
  if (!ensureClient() || !client_) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(client)", "client not available");
    done_.store(true);
    success_.store(false);
    return;
  }

  if (!client_->wait_for_service(2s)) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(service)", "service not available");
    done_.store(true);
    success_.store(false);
    return;
  }

  auto req = std::make_shared<Srv::Request>();
  req->arm_id = arm_id;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    future_ = client_->async_send_request(req);
  }
  req_sent_.store(true);
}

BT::NodeStatus GraspgenEnable::onStart()
{
  resetState();

  // optional override
  (void)getInput("service_name", service_name_);

  // service_name 바뀌면 client 재생성
  client_.reset();
  ensureClient();

  std::string arm_id;
  if (!getInput("arm_id", arm_id) || arm_id.empty()) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(arm_id)", "missing arm_id");
    return BT::NodeStatus::FAILURE;
  }

  if (!started_log_once_.exchange(true)) {
    RCLCPP_INFO(node_->get_logger(),
                "[BT] %s : call service %s (arm_id=%s)",
                this->name().c_str(),
                service_name_.c_str(),
                arm_id.c_str());
  }

  sendRequest(arm_id);

  if (done_.load()) {
    BT_LOG_STATUS(node_->get_logger(), this->name(),
                  success_.load() ? "SUCCESS" : "FAILURE",
                  "out(result)");
    return success_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GraspgenEnable::onRunning()
{
  if (done_.load()) {
    BT_LOG_STATUS(node_->get_logger(), this->name(),
                  success_.load() ? "SUCCESS" : "FAILURE",
                  "out(result)");
    return success_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  Client::SharedFuture fut;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    fut = future_;
  }

  if (!req_sent_.load() || !fut.valid()) {
    if (!running_wait_log_once_.exchange(true)) {
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(future)");
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto st = fut.wait_for(0ms);
  if (st != std::future_status::ready) {
    if (!running_wait_log_once_.exchange(true)) {
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(service)");
    }
    return BT::NodeStatus::RUNNING;
  }

  try {
    auto resp = fut.get();

    if (resp->success) {
      done_.store(true);
      success_.store(true);
      BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "out(service)", resp->message);
      return BT::NodeStatus::SUCCESS;
    } else {
      done_.store(true);
      success_.store(false);
      BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(service)", resp->message);
      return BT::NodeStatus::FAILURE;
    }
  } catch (const std::exception& e) {
    done_.store(true);
    success_.store(false);
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(exception)", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void GraspgenEnable::onHalted()
{
  if (node_) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "HALTED", "noop");
  }
  // 서비스는 cancel 개념이 없으므로 상태만 초기화
  resetState();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<GraspgenEnable>("GraspgenEnable");
}

} // namespace GraspgenEnable
