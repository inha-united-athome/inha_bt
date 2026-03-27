// Waving.cpp
#include "inha_bt_pkg/perception/Waving.h"
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

namespace Waving
{

Waving::Waving(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("@node");
  if (!node_) {
    throw BT::RuntimeError("Waving: missing 'node' in blackboard");
  }
  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "got node from blackboard");
}

void Waving::ensureClient()
{
  if (client_) return;

  client_ = rclcpp_action::create_client<WavingAction>(node_, kActionName);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "READY", "out(action)",
                std::string("client created: ") + kActionName);
}

BT::NodeStatus Waving::onStart()
{
  ensureClient();

  printed_waiting_ = false;
  done_.store(false);
  ok_.store(false);
  active_.store(true);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_handle_.reset();
    last_error_.clear();
  }

  BT_LOG_STATUS(node_->get_logger(), this->name(), "START", "out(action)", "send goal(start=true)");

  if (!client_->wait_for_action_server(300ms)) {
    active_.store(false);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_error_ = std::string("action server not available: ") + kActionName;
    }
    setOutput("error_message", last_error_);
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(action)", last_error_);
    return BT::NodeStatus::FAILURE;
  }

  WavingAction::Goal goal;
  goal.start = true;

  rclcpp_action::Client<WavingAction>::SendGoalOptions options;

  options.goal_response_callback =
    [this](GoalHandleWaving::SharedPtr gh)
    {
      if (!gh) {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          last_error_ = "goal rejected";
        }
        done_.store(true);
        ok_.store(false);
        active_.store(false);
        BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(goal_response)", "goal rejected");
        return;
      }
      {
        std::lock_guard<std::mutex> lk(mtx_);
        goal_handle_ = gh;
      }
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "in(goal_response)", "goal accepted");
    };

  options.result_callback =
    [this](const GoalHandleWaving::WrappedResult& result)
    {
      bool success_flag = false;
      std::string err;

      // ✅ 네 액션 정의: Result{ bool success; string error_message; }
      if (result.result) {
        success_flag = result.result->success;
        err = result.result->error_message;
      } else {
        success_flag = false;
        err = "null result pointer";
      }

      {
        std::lock_guard<std::mutex> lk(mtx_);
        last_error_ = err;
      }

      ok_.store(success_flag);
      done_.store(true);
      active_.store(false);
    };

  (void)client_->async_send_goal(goal, options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Waving::onRunning()
{

  if (!done_.load()) {
    if (!printed_waiting_) {
      printed_waiting_ = true;
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(result)");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (ok_.load()) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "in(result)", "success=true");
    return BT::NodeStatus::SUCCESS;
  }

  std::string err;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    err = last_error_.empty() ? "success=false" : last_error_;
  }
  setOutput("error_message", err);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(result)", err);
  return BT::NodeStatus::FAILURE;
}

void Waving::onHalted()
{
  active_.store(false);

  GoalHandleWaving::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    gh = goal_handle_;
  }

  if (client_ && gh) {
    (void)client_->async_cancel_goal(gh);
    BT_LOG_STATUS(node_->get_logger(), this->name(), "HALTED", "out(cancel)", "cancel requested");
  } else {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "HALTED", "out(cancel)", "no active goal_handle");
  }
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<Waving>("Waving");
}

} // namespace Waving
