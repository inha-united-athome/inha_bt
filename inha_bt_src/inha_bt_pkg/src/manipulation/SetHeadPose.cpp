#include "inha_bt_pkg/manipulation/SetHeadPose.h"

#include <chrono>

using namespace std::chrono_literals;

namespace SetHeadPose
{

SetHeadPoseBT::SetHeadPoseBT(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("@node");
  if (!node_) {
    throw BT::RuntimeError("SetHeadPoseBT: missing 'node' in blackboard");
  }
}

void SetHeadPoseBT::resetState()
{
  done_.store(false, std::memory_order_relaxed);
  ok_.store(false, std::memory_order_relaxed);

  std::lock_guard<std::mutex> lk(mtx_);
  goal_handle_.reset();
  error_message_.clear();

  (void)setOutput("error_message", std::string(""));
}

bool SetHeadPoseBT::ensureClient()
{
  if (!client_) {
    client_ = rclcpp_action::create_client<ActionT>(node_, action_name_);
  }
  return client_->wait_for_action_server(500ms);
}

BT::NodeStatus SetHeadPoseBT::onStart()
{
  resetState();

  const std::string requested_action_name = getInput<std::string>("action_name").value_or("/rby1/set_head_pose");

  if (!client_ || requested_action_name != action_name_) {
    client_ = rclcpp_action::create_client<ActionT>(node_, requested_action_name);
  }
  action_name_ = requested_action_name;

  if (!ensureClient()) {
    std::lock_guard<std::mutex> lk(mtx_);
    error_message_ = std::string("action server not available: ") + action_name_;
    (void)setOutput("error_message", error_message_);
    return BT::NodeStatus::FAILURE;
  }

  const double head_0 = getInput<double>("head_0").value_or(0.0);
  const double head_1 = getInput<double>("head_1").value_or(0.0);
  const double duration = getInput<double>("duration").value_or(1.0);

  ActionT::Goal goal;
  goal.head_0 = head_0;
  goal.head_1 = head_1;
  goal.duration = duration;

  rclcpp_action::Client<ActionT>::SendGoalOptions opts;

  opts.goal_response_callback =
    [this](GoalHandle::SharedPtr gh)
    {
      if (!gh) {
        std::lock_guard<std::mutex> lk(mtx_);
        error_message_ = "goal rejected";
        ok_.store(false, std::memory_order_relaxed);
        done_.store(true, std::memory_order_relaxed);
        return;
      }

      std::lock_guard<std::mutex> lk(mtx_);
      goal_handle_ = gh;
    };

  opts.result_callback =
    [this](const GoalHandle::WrappedResult& wr)
    {
      bool ok = false;
      std::string err;

      if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result) {
        ok = wr.result->success;
        err = wr.result->error_message;
      } else {
        err = std::string("result_code=") + std::to_string(static_cast<int>(wr.code));
      }

      {
        std::lock_guard<std::mutex> lk(mtx_);
        error_message_ = err;
      }

      ok_.store(ok, std::memory_order_relaxed);
      done_.store(true, std::memory_order_relaxed);
    };

  (void)client_->async_send_goal(goal, opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetHeadPoseBT::onRunning()
{
  if (!done_.load(std::memory_order_relaxed)) {
    return BT::NodeStatus::RUNNING;
  }

  std::string err;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    err = error_message_;
  }
  (void)setOutput("error_message", err);

  return ok_.load(std::memory_order_relaxed) ? BT::NodeStatus::SUCCESS
                                              : BT::NodeStatus::FAILURE;
}

void SetHeadPoseBT::onHalted()
{
  GoalHandle::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    gh = goal_handle_;
  }

  if (client_ && gh) {
    (void)client_->async_cancel_goal(gh);
  }

  resetState();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetHeadPoseBT>("SetHeadPose");
}

}  // namespace SetHeadPose
