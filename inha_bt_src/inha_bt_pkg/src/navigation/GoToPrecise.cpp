#include "inha_bt_pkg/navigation/GoToPrecise.h"

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

namespace GoToPrecise
{

GoToPrecise::GoToPrecise(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("GoToPrecise: missing 'node' in blackboard");
  }
  ensureClient();
}

BT::PortsList GoToPrecise::providedPorts()
{
  return {
    // goal pose
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "target goal pose"),

    // optional overrides
    BT::InputPort<std::string>("action_name", std::string("/navigate_to_pose"), "nav2 action name"),
    BT::InputPort<std::string>("behavior_tree", std::string("goal_precise.xml"), "nav2 behavior tree xml"),

    // error output
    BT::OutputPort<std::string>("error_message", "error message")
  };
}

void GoToPrecise::setLastError(const std::string& msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_error_ = msg;
}

std::string GoToPrecise::getLastErrorCopy() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return last_error_;
}

void GoToPrecise::resetState()
{
  started_log_once_.store(false);
  running_wait_log_once_.store(false);

  goal_sent_.store(false);
  done_.store(false);
  success_.store(false);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_handle_.reset();
    last_error_.clear();
  }

  // tick thread에서만 포트 조작
  setOutput("error_message", std::string(""));
}

bool GoToPrecise::ensureClient()
{
  if (!node_) {
    return false;
  }

  if (!client_) {
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name_);
  }
  return (client_ != nullptr);
}

void GoToPrecise::sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
{
  if (!ensureClient() || !client_) {
    setLastError("GoToPrecise: action client not available");
    done_.store(true);
    success_.store(false);
    return;
  }

  if (!client_->wait_for_action_server(2s)) {
    setLastError("GoToPrecise: action server not available: " + action_name_);
    done_.store(true);
    success_.store(false);
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_pose;
  goal_msg.behavior_tree = behavior_tree_;  // ✅ BT 지정

  typename ClientNav::SendGoalOptions opts;

  opts.goal_response_callback =
    [this](GoalHandleNav::SharedPtr gh)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal_handle_ = gh;

      if (!gh) {
        last_error_ = "GoToPrecise: goal rejected";
        done_.store(true);
        success_.store(false);
      } else {
        goal_sent_.store(true);
      }
    };

  opts.feedback_callback =
    [this](GoalHandleNav::SharedPtr /*gh*/,
           const std::shared_ptr<const NavigateToPose::Feedback> /*fb*/)
    {
      (void)this;
    };

  opts.result_callback =
    [this](const GoalHandleNav::WrappedResult& result)
    {
      bool ok = false;
      std::string err;

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          ok = true;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          ok = false;
          err = "GoToPrecise: aborted";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          ok = false;
          err = "GoToPrecise: canceled";
          break;
        default:
          ok = false;
          err = "GoToPrecise: unknown result code";
          break;
      }

      if (!ok && err.empty()) {
        err = "GoToPrecise: failed";
      }

      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!ok) {
          last_error_ = err;
        }
      }

      done_.store(true);
      success_.store(ok);
    };

  client_->async_send_goal(goal_msg, opts);
}

void GoToPrecise::cancelGoal()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!client_ || !goal_handle_) {
    return;
  }
  (void)client_->async_cancel_goal(goal_handle_);
}

BT::NodeStatus GoToPrecise::onStart()
{
  resetState();

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!getInput("goal", goal_pose)) {
    setLastError("GoToPrecise: missing input port 'goal'");
    setOutput("error_message", getLastErrorCopy());
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(goal)", "missing goal");
    return BT::NodeStatus::FAILURE;
  }

  // optional overrides
  std::string new_action = action_name_;
  std::string new_bt = behavior_tree_;
  (void)getInput("action_name", new_action);
  (void)getInput("behavior_tree", new_bt);

  // action_name 바뀌면 client 재생성
  if (new_action != action_name_) {
    action_name_ = new_action;
    client_.reset();
  }
  behavior_tree_ = new_bt;

  ensureClient();

  if (!started_log_once_.exchange(true)) {
    RCLCPP_INFO(node_->get_logger(),
                "[BT] %s : sending goal to %s (behavior_tree=%s)",
                this->name().c_str(),
                action_name_.c_str(),
                behavior_tree_.c_str());
  }

  sendGoal(goal_pose);

  // 즉시 실패/성공 처리
  if (done_.load()) {
    if (success_.load()) {
      BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "out(result)");
      return BT::NodeStatus::SUCCESS;
    } else {
      const auto err = getLastErrorCopy();
      setOutput("error_message", err);
      BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(result)", err);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPrecise::onRunning()
{
  if (!done_.load()) {
    if (!running_wait_log_once_.exchange(true)) {
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(result)");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (success_.load()) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "out(result)");
    return BT::NodeStatus::SUCCESS;
  }

  const auto err = getLastErrorCopy();
  setOutput("error_message", err);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(result)", err);
  return BT::NodeStatus::FAILURE;
}

void GoToPrecise::onHalted()
{
  if (node_) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "HALTED", "cancel");
  }
  cancelGoal();
  resetState();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<GoToPrecise>("GoToPrecise");
}

} // namespace GoToPrecise
