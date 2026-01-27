#include <chrono>
#include <future>

#include "inha_bt_pkg/manipulation/SetRobotPose.h"

using namespace std::chrono_literals;

namespace SetRobotPose
{

SetRobotPoseBT::SetRobotPoseBT(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = rclcpp_action::create_client<ActionT>(node_, "/rby1/set_pose");
}

BT::PortsList SetRobotPoseBT::providedPorts()
{
  return {
    BT::InputPort<int>("mode", "pose mode"),
    BT::InputPort<double>("delay_sec", 0.0, "delay seconds")
  };
}

BT::NodeStatus SetRobotPoseBT::onStart()
{
  done_ = false;
  ok_ = false;
  goal_handle_.reset();

  if (!client_->wait_for_action_server(500ms)) return BT::NodeStatus::FAILURE;

  int mode = 0;
  if (!getInput("mode", mode)) return BT::NodeStatus::FAILURE;

  double delay = 0.0;
  (void)getInput("delay_sec", delay);

  ActionT::Goal goal;
  goal.mode = mode;
  goal.delay_sec = static_cast<float>(delay);

  rclcpp_action::Client<ActionT>::SendGoalOptions opt;
  opt.result_callback =
    [this](const GoalHandle::WrappedResult& wr)
    {
      std::lock_guard<std::mutex> lk(mx_);
      done_ = true;
      ok_ = (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result && wr.result->success);
    };

  auto fut = client_->async_send_goal(goal, opt);
  if (fut.wait_for(200ms) != std::future_status::ready) return BT::NodeStatus::FAILURE;

  goal_handle_ = fut.get();
  if (!goal_handle_) return BT::NodeStatus::FAILURE;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetRobotPoseBT::onRunning()
{
  if (!done_) return BT::NodeStatus::RUNNING;
  return ok_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void SetRobotPoseBT::onHalted()
{
  if (goal_handle_) (void)client_->async_cancel_goal(goal_handle_);
  done_ = true;
  ok_ = false;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetRobotPoseBT>("SetRobotPose");
}

}  // namespace SetRobotPose
