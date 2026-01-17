#include "inha_bt_pkg/navigation/GoToPose.h"

namespace GoToPose
{

// ---- 공통: 요약 로그 헬퍼 ----
static inline void BT_LOG_STATUS(
  const rclcpp::Logger& logger,
  const std::string& node_name,
  const char* status,
  const std::string& io,
  const std::string& reason = "")
{
  if (!reason.empty()) {
    RCLCPP_WARN(logger, "[BT] %s : %s  (%s)  reason=%s",
                node_name.c_str(), status, io.c_str(), reason.c_str());
  } else {
    RCLCPP_INFO(logger, "[BT] %s : %s  (%s)",
                node_name.c_str(), status, io.c_str());
  }
}

static inline std::string goalToStr(const geometry_msgs::msg::PoseStamped& g)
{
  const auto& p = g.pose.position;
  const auto& q = g.pose.orientation;
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "frame=%s pos=(%.2f,%.2f,%.2f) quat=(%.3f,%.3f,%.3f,%.3f)",
                g.header.frame_id.c_str(), p.x, p.y, p.z, q.x, q.y, q.z, q.w);
  return std::string(buf);
}

// ----------------- GoToPose -----------------
GoToPose::GoToPose(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 생성로그는 원하면 남기고, 아니면 빼도 됨(스팸 줄이려면 주석)
  // BT_LOG_STATUS(node_->get_logger(), name, "READY", "action:/navigate_to_pose");
}

BT::PortsList GoToPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::string>("action_name", "/navigate_to_pose")
  };
}

void GoToPose::resetState()
{
  goal_sent_.store(false, std::memory_order_relaxed);
  done_.store(false, std::memory_order_relaxed);
  success_.store(false, std::memory_order_relaxed);

  // 스팸 방지용 플래그
  running_wait_log_once_.store(false, std::memory_order_relaxed);

  std::lock_guard<std::mutex> lk(mtx_);
  goal_handle_.reset();
}

bool GoToPose::ensureClient()
{
  if (!client_) {
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name_);
  }
  return client_->wait_for_action_server(std::chrono::milliseconds(200));
}

void GoToPose::sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
{
  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_pose;

  done_.store(false, std::memory_order_relaxed);
  success_.store(false, std::memory_order_relaxed);

  // "현재 노드가 하는 일" = action goal 보내기
  BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING",
                std::string("action:") + action_name_ + " send_goal " + goalToStr(goal_pose));

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

  options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr handle)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal_handle_ = handle;

      if (!goal_handle_) {
        done_.store(true, std::memory_order_relaxed);
        success_.store(false, std::memory_order_relaxed);

        BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE",
                      std::string("action:") + action_name_,
                      "goal rejected by server");
        return;
      }
      // ACCEPTED는 성공/실패 최종이 아니라서 굳이 안 찍음(원하면 여기 INFO 한 줄 추가 가능)
    };

  options.result_callback =
    [this](const GoalHandleNav::WrappedResult& result)
    {
      done_.store(true, std::memory_order_relaxed);

      std::string reason;
      bool ok = false;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          ok = true; reason = ""; break;
        case rclcpp_action::ResultCode::ABORTED:
          ok = false; reason = "action result: ABORTED"; break;
        case rclcpp_action::ResultCode::CANCELED:
          ok = false; reason = "action result: CANCELED"; break;
        default:
          ok = false; reason = "action result: UNKNOWN"; break;
      }

      success_.store(ok, std::memory_order_relaxed);

      if (ok) {
        BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS",
                      std::string("action:") + action_name_);
      } else {
        BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE",
                      std::string("action:") + action_name_, reason);
      }
    };

  client_->async_send_goal(goal_msg, options);
  goal_sent_.store(true, std::memory_order_relaxed);
}

void GoToPose::cancelGoal()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (client_ && goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
}

BT::NodeStatus GoToPose::onStart()
{
  resetState();

  action_name_ = getInput<std::string>("action_name").value_or("/navigate_to_pose");

  auto goal_any = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_any) {
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE",
                  std::string("action:") + action_name_,
                  "missing input port 'goal'");
    return BT::NodeStatus::FAILURE;
  }

  if (!ensureClient()) {
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE",
                  std::string("action:") + action_name_,
                  "action server not available");
    return BT::NodeStatus::FAILURE;
  }

  sendGoal(goal_any.value());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  if (!goal_sent_.load(std::memory_order_relaxed)) {
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE",
                  std::string("action:") + action_name_,
                  "internal error: goal_sent_=false");
    return BT::NodeStatus::FAILURE;
  }

  if (!done_.load(std::memory_order_relaxed)) {
    // 대기중 로그는 1번만(현재 진행중 + IO만)
    bool expected = false;
    if (running_wait_log_once_.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
      BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING",
                    std::string("action:") + action_name_ + " waiting_result");
    }
    return BT::NodeStatus::RUNNING;
  }

  return success_.load(std::memory_order_relaxed) ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
}

void GoToPose::onHalted()
{
  cancelGoal();
  resetState();
  BT_LOG_STATUS(node_->get_logger(), name(), "HALTED", std::string("action:") + action_name_);
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<GoToPose>("GoToPose");
}

} // namespace GoToPose
