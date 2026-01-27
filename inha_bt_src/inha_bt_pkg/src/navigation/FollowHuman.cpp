#include "inha_bt_pkg/navigation/FollowHuman.h"

#include <chrono>

using namespace std::chrono_literals;

namespace FollowHuman
{

FollowHumanBT::FollowHumanBT(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("FollowHumanBT: missing blackboard entry [node]");
  }

  action_name_ = "nav/follow_human";
}

BT::NodeStatus FollowHumanBT::onStart()
{
  goal_handle_.reset();

  client_ = rclcpp_action::create_client<FollowHumanAction>(node_, action_name_);

  if (!client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "FollowHuman action server not available");
    return BT::NodeStatus::FAILURE;
  }

  FollowHumanAction::Goal goal;
  goal.enable = true;   // ✅ action Goal

  goal_future_ = client_->async_send_goal(goal);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowHumanBT::onRunning()
{
  // 아직 goal handle 안 왔으면 대기
  if (!goal_handle_) {
    if (goal_future_.valid() &&
        goal_future_.wait_for(0ms) == std::future_status::ready)
    {
      goal_handle_ = goal_future_.get();
      if (!goal_handle_) {
        return BT::NodeStatus::FAILURE;
      }
      result_future_ = client_->async_get_result(goal_handle_);
    }
    return BT::NodeStatus::RUNNING;
  }

  // 결과 대기
  if (result_future_.valid() &&
      result_future_.wait_for(0ms) == std::future_status::ready)
  {
    auto wrapped = result_future_.get();

    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      return BT::NodeStatus::FAILURE;
    }

    return wrapped.result->success ? BT::NodeStatus::SUCCESS
                                   : BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void FollowHumanBT::onHalted()
{
  if (!client_) return;

  if (!client_->wait_for_action_server(1000ms)) {
    RCLCPP_WARN(node_->get_logger(), "FollowHuman action server not available on halt");
    return;
  }

  FollowHumanAction::Goal stop_goal;
  stop_goal.enable = false;

  // stop goal은 fire-and-forget: 결과 기다리면 halt에서 BT가 멈칫할 수 있으니 비동기로만 던짐
  (void)client_->async_send_goal(stop_goal);

  // (선택) 다음 onStart 때 꼬이지 않게 핸들만 비워둠
  goal_handle_.reset();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<FollowHumanBT>("FollowHuman");
}

}  // namespace FollowHuman
