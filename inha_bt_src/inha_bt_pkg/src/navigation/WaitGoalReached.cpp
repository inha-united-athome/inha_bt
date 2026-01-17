#include "inha_bt_pkg/navigation/WaitGoalReached.h"
#include <behaviortree_cpp/bt_factory.h>

namespace WaitGoalReached
{

static const char* statusToStr(uint8_t s)
{
  using action_msgs::msg::GoalStatus;
  switch (s) {
    case GoalStatus::STATUS_UNKNOWN:   return "UNKNOWN";
    case GoalStatus::STATUS_ACCEPTED:  return "ACCEPTED";
    case GoalStatus::STATUS_EXECUTING: return "EXECUTING";
    case GoalStatus::STATUS_CANCELING: return "CANCELING";
    case GoalStatus::STATUS_SUCCEEDED: return "SUCCEEDED";
    case GoalStatus::STATUS_CANCELED:  return "CANCELED";
    case GoalStatus::STATUS_ABORTED:   return "ABORTED";
    default:                           return "?(invalid)";
  }
}

// ---- 요약 로그 헬퍼 ----
static inline void BT_SUMMARY(
  const rclcpp::Logger& logger,
  const std::string& node_name,
  const char* state,
  const char* io,
  const std::string& reason = "")
{
  if (!reason.empty()) {
    RCLCPP_WARN(logger, "[BT] %s : %s  (%s)  reason=%s",
                node_name.c_str(), state, io, reason.c_str());
  } else {
    RCLCPP_INFO(logger, "[BT] %s : %s  (%s)",
                node_name.c_str(), state, io);
  }
}

static constexpr const char* IO_STATUS =
  "sub:/navigate_to_pose/_action/status";

WaitGoalReached::WaitGoalReached(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  clock_ = node_->get_clock();

  sub_status_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
    "/navigate_to_pose/_action/status", 10,
    [this](action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
      onStatus_(std::move(msg));
    });

  // 생성 로그는 필요하면 살리고, 스팸 줄이려면 주석
  // BT_SUMMARY(node_->get_logger(), name, "READY", IO_STATUS);
}

BT::PortsList WaitGoalReached::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_sec", 60.0, "timeout seconds"),
    BT::InputPort<rclcpp::Node::SharedPtr>("node")
  };
}

void WaitGoalReached::onStatus_(action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(m_);
  if (!active_) return;

  // 1) target goal stamp 아직이면: start_time_ 이후 가장 최신 goal 선택
  if (!target_set_)
  {
    bool found = false;
    int64_t newest_ns = 0;

    for (const auto& st : msg->status_list)
    {
      const rclcpp::Time goal_stamp(st.goal_info.stamp, clock_->get_clock_type());
      if (goal_stamp < start_time_) continue;

      const int64_t ns = goal_stamp.nanoseconds();
      if (!found || ns > newest_ns)
      {
        found = true;
        newest_ns = ns;
      }
    }

    if (!found) return;

    target_set_ = true;
    target_stamp_ns_ = newest_ns;
    last_status_ = -1;

    // target 잡는 순간 1번만 요약 로그
    BT_SUMMARY(node_->get_logger(), name(), "RUNNING", IO_STATUS,
               std::string("target selected (stamp_ns=") + std::to_string(target_stamp_ns_) + ")");
  }

  // 2) 선택된 target stamp의 status만 추적
  for (const auto& st : msg->status_list)
  {
    const rclcpp::Time goal_stamp(st.goal_info.stamp, clock_->get_clock_type());
    if (goal_stamp.nanoseconds() != target_stamp_ns_) continue;

    const uint8_t s = st.status;

    // 상태 변화 기록은 내부 변수로만 유지(스팸 방지)
    last_status_ = static_cast<int8_t>(s);

    if (s == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    {
      done_ = true;
      success_ = true;
    }
    else if (s == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
             s == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      done_ = true;
      success_ = false;
      fail_reason_ = std::string("status=") + statusToStr(s);
    }
    break;
  }
}

BT::NodeStatus WaitGoalReached::onStart()
{
  double tmo = 60.0;
  getInput("timeout_sec", tmo);

  std::lock_guard<std::mutex> lk(m_);
  timeout_sec_ = std::max(0.0, tmo);
  start_time_ = clock_->now();

  active_ = true;
  done_ = false;
  success_ = false;

  target_set_ = false;
  target_stamp_ns_ = 0;

  last_status_ = -1;
  printed_wait_ = false;
  fail_reason_.clear();

  // onStart는 RUNNING 1번만
  BT_SUMMARY(node_->get_logger(), name(), "RUNNING", IO_STATUS,
             std::string("start (timeout=") + std::to_string(timeout_sec_) + "s)");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitGoalReached::onRunning()
{
  std::lock_guard<std::mutex> lk(m_);

  // 아직 target 못 잡았으면 "waiting" 1번만
  if (!target_set_ && !printed_wait_)
  {
    printed_wait_ = true;
    BT_SUMMARY(node_->get_logger(), name(), "RUNNING", IO_STATUS,
               "waiting first status after start_time");
  }

  // timeout 체크
  if (timeout_sec_ > 0.0)
  {
    const double elapsed = (clock_->now() - start_time_).seconds();
    if (elapsed > timeout_sec_)
    {
      active_ = false;
      BT_SUMMARY(node_->get_logger(), name(), "FAILURE", IO_STATUS,
                 std::string("timeout (elapsed=") + std::to_string(elapsed) + "s)");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 완료되면 최종 1번만
  if (done_)
  {
    active_ = false;

    if (success_) {
      BT_SUMMARY(node_->get_logger(), name(), "SUCCESS", IO_STATUS);
      return BT::NodeStatus::SUCCESS;
    } else {
      std::string reason = !fail_reason_.empty() ? fail_reason_
                                                 : std::string("status=") + statusToStr(static_cast<uint8_t>(last_status_));
      BT_SUMMARY(node_->get_logger(), name(), "FAILURE", IO_STATUS, reason);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void WaitGoalReached::onHalted()
{
  std::lock_guard<std::mutex> lk(m_);

  active_ = false;
  done_ = false;
  success_ = false;

  target_set_ = false;
  target_stamp_ns_ = 0;

  last_status_ = -1;
  printed_wait_ = false;
  fail_reason_.clear();

  BT_SUMMARY(node_->get_logger(), name(), "HALTED", IO_STATUS);
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<WaitGoalReached>("WaitGoalReached");
}

} // namespace WaitGoalReached
