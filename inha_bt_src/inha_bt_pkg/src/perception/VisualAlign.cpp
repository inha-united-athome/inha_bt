// VisualAlign.cpp
#include "inha_bt_pkg/perception/VisualAlign.h"
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

namespace VisualAlign
{

VisualAlign::VisualAlign(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("VisualAlign: missing 'node' in blackboard");
  }
  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "got node from blackboard");
}

void VisualAlign::ensureClient()
{
  if (client_) return;

  client_ = rclcpp_action::create_client<VisualAlignAction>(node_, kActionName);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "READY", "out(action)",
                std::string("client created: ") + kActionName);
}

BT::NodeStatus VisualAlign::onStart()
{
  ensureClient();

  done_.store(false);
  ok_.store(false);
  printed_waiting_.store(false);
  last_error_.clear();
  goal_handle_.reset();

  bool enable = true;
  (void)getInput("enable", enable);

  BT_LOG_STATUS(node_->get_logger(), this->name(), "START", "out(action)",
                std::string("send goal(enable=") + (enable ? "true" : "false") + ")");

  if (!client_->wait_for_action_server(300ms)) {
    last_error_ = std::string("action server not available: ") + kActionName;
    setOutput("error_message", last_error_);
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(action)", last_error_);
    return BT::NodeStatus::FAILURE;
  }

  VisualAlignAction::Goal goal;
  goal.enable = enable;

  rclcpp_action::Client<VisualAlignAction>::SendGoalOptions options;

  options.goal_response_callback =
    [this](GoalHandleVA::SharedPtr gh)
    {
      if (!gh) {
        last_error_ = "goal rejected";
        ok_.store(false);
        done_.store(true);
        BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(goal_response)", "goal rejected");
        return;
      }
      goal_handle_ = gh;  // halt cancel용(원치 않으면 이 줄도 제거 가능)
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "in(goal_response)", "goal accepted");
    };

  options.result_callback =
    [this](const GoalHandleVA::WrappedResult& result)
    {
      bool success_flag = false;
      std::string err;

      // 네 정의: Result{ bool success; string error_message; }
      if (result.result) {
        success_flag = result.result->success;
        err = result.result->error_message;
      } else {
        success_flag = false;
        err = "null result pointer";
      }

      last_error_ = err;
      ok_.store(success_flag);
      done_.store(true);
    };

  (void)client_->async_send_goal(goal, options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VisualAlign::onRunning()
{
  if (!done_.load()) {
    if (!printed_waiting_.load()) {
      printed_waiting_.store(true);
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(result)");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (ok_.load()) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "in(result)", "success=true");
    return BT::NodeStatus::SUCCESS;
  }

  std::string err = last_error_.empty() ? "success=false" : last_error_;
  setOutput("error_message", err);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(result)", err);
  return BT::NodeStatus::FAILURE;
}

void VisualAlign::onHalted()
{
  if (client_ && goal_handle_) {
    (void)client_->async_cancel_goal(goal_handle_);
    BT_LOG_STATUS(node_->get_logger(), this->name(), "HALTED", "out(cancel)", "cancel requested");
  } else {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "HALTED", "out(cancel)", "no active goal_handle");
  }
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<VisualAlign>("VisualAlign");
}

} // namespace VisualAlign
