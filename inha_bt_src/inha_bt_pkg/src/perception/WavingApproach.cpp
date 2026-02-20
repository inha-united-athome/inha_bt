// WavingApproach.cpp
#include "inha_bt_pkg/perception/WavingApproach.h"

#include <chrono>
using namespace std::chrono_literals;

namespace
{
// ---- 공통: 요약 로그 헬퍼 ----
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

namespace WavingApproach
{

WavingApproach::WavingApproach(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("WavingApproach: missing 'node' in blackboard");
  }
  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "got node from blackboard");
}

void WavingApproach::ensureClient()
{
  if (client_) return;
  client_ = rclcpp_action::create_client<ApproachAction>(node_, kActionName);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "READY", "out(action)",
                std::string("client created: ") + kActionName);
}

BT::NodeStatus WavingApproach::onStart()
{
  ensureClient();

  printed_waiting_ = false;
  done_.store(false);
  ok_.store(false);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_handle_.reset();
    goal_pose_ = geometry_msgs::msg::PoseStamped{};
  }

  BT_LOG_STATUS(node_->get_logger(), this->name(), "START", "out(action)", "send goal(start=true)");

  if (!client_->wait_for_action_server(300ms)) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(action)",
                  std::string("action server not available: ") + kActionName);
    return BT::NodeStatus::FAILURE;
  }

  // ---- send trigger goal ----
  ApproachAction::Goal goal_msg;
  goal_msg.start = true;  // ✅ Goal: bool start

  rclcpp_action::Client<ApproachAction>::SendGoalOptions opt;

  opt.goal_response_callback =
    [this](GoalHandleApproach::SharedPtr gh)
    {
      if (!gh) {
        ok_.store(false);
        done_.store(true);
        return;
      }
      {
        std::lock_guard<std::mutex> lk(mtx_);
        goal_handle_ = gh;
      }
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "in(goal_response)", "goal accepted");
    };

  opt.result_callback =
    [this](const GoalHandleApproach::WrappedResult& res)
    {
        bool ok = false;
        geometry_msgs::msg::PoseStamped fp;

        // 1) 액션 레벨 결과 코드 체크
        if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
        ok = false;
        } else if (res.result) {
        // 2) 너희 Result.success 체크
        ok = res.result->success;
        fp = res.result->goal_pose;
        } else {
        ok = false;
        }

        {
        std::lock_guard<std::mutex> lk(mtx_);
        goal_pose_ = fp;
        }

        ok_.store(ok);
        done_.store(true);
    };


  (void)client_->async_send_goal(goal_msg, opt);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WavingApproach::onRunning()
{

  if (!done_.load()) {
    if (!printed_waiting_) {
      printed_waiting_ = true;
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(result)");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (ok_.load()) {
    geometry_msgs::msg::PoseStamped fp;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      fp = goal_pose_;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[BT] %s : goal_pose frame=%s pos=(%.2f, %.2f, %.2f)",
      this->name().c_str(),
      fp.header.frame_id.c_str(),
      fp.pose.position.x,
      fp.pose.position.y,
      fp.pose.position.z
    );

    setOutput("goal_pose", fp);

    BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "in(result)", "success=true");
    return BT::NodeStatus::SUCCESS;
  }

  BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(result)", "success=false");
  return BT::NodeStatus::FAILURE;
}


void WavingApproach::onHalted()
{
  GoalHandleApproach::SharedPtr gh;
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
  factory.registerNodeType<WavingApproach>("WavingApproach");
}

} // namespace WavingApproach
