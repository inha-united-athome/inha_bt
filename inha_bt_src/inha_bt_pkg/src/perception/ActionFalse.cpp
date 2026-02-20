// ===== src/perception/ActionFalse.cpp =====
#include "inha_bt_pkg/perception/ActionFalse.h"

#include <chrono>

namespace ActionFalse
{

ActionFalse::ActionFalse(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  ensureClient();
}

void ActionFalse::ensureClient()
{
  if (!client_) {
    client_ = rclcpp_action::create_client<Act>(node_, kActionName);
  }
}

BT::NodeStatus ActionFalse::onStart()
{
  ensureClient();

  // 상태 초기화
  active_.store(true);
  done_.store(false);
  ok_.store(false);
  printed_waiting_ = false;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_handle_.reset();
    last_error_.clear();
  }
  setOutput("error_message", std::string(""));

  // ✅ 서버 없으면 그냥 무시하고 SUCCESS
  if (!client_->wait_for_action_server(std::chrono::milliseconds(300))) {
    RCLCPP_WARN(node_->get_logger(), "[BT] ActionFalse: server not available -> ignore");
    active_.store(false);
    done_.store(true);
    ok_.store(true);
    return BT::NodeStatus::SUCCESS;
  }

  // ✅ Goal은 무조건 false로 보냄
  Act::Goal goal;
  goal.start = false;

  rclcpp_action::Client<Act>::SendGoalOptions opts;

  // accept/reject는 그냥 로그만 (BT 성공 처리에는 영향 없음)
  opts.goal_response_callback =
    [this](GoalHandle::SharedPtr gh)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal_handle_ = gh;

      if (!gh) {
        RCLCPP_WARN(node_->get_logger(), "[BT] ActionFalse: goal rejected -> ignore");
      } else {
        RCLCPP_INFO(node_->get_logger(), "[BT] ActionFalse: goal accepted");
      }
    };

  // 결과도 그냥 로그만
  opts.result_callback =
    [this](const GoalHandle::WrappedResult& wr)
    {
      if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(node_->get_logger(),
                    "[BT] ActionFalse: result code=%d (ignored)",
                    (int)wr.code);
      } else {
        RCLCPP_INFO(node_->get_logger(), "[BT] ActionFalse: result SUCCEEDED (ignored)");
      }
    };

  (void)client_->async_send_goal(goal, opts);

  // ✅ 즉시 성공 처리
  active_.store(false);
  done_.store(true);
  ok_.store(true);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ActionFalse::onRunning()
{
  // onStart에서 바로 SUCCESS로 끝나므로 혹시 불려도 SUCCESS
  return BT::NodeStatus::SUCCESS;
}

void ActionFalse::onHalted()
{
  // halt되어도 실패 처리 안 함
  active_.store(false);
  done_.store(true);
  ok_.store(true);
  setOutput("error_message", std::string(""));
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ActionFalse>("ActionFalse");
}

} // namespace ActionFalse
