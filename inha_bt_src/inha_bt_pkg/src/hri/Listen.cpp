#include "inha_bt_pkg/hri/Listen.h"

namespace Listen
{

static inline void BT_LOG_STATUS(
  const rclcpp::Logger& logger,
  const std::string& node_name,
  const char* status,
  const std::string& io,
  const std::string& reason = "")
{
  if (!reason.empty()) {
    RCLCPP_WARN(logger, "[BT] %s : %s (%s) reason=%s",
                node_name.c_str(), status, io.c_str(), reason.c_str());
  } else {
    RCLCPP_INFO(logger, "[BT] %s : %s (%s)",
                node_name.c_str(), status, io.c_str());
  }
}

Listen::Listen(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = rclcpp_action::create_client<ListenAction>(node_, "listen");
}

BT::PortsList Listen::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node"),
    BT::InputPort<float>("start_timeout_sec", 5.0f, "Wait speech start timeout (sec)"),
    BT::InputPort<std::string>("action_name", "listen", "Listen action name"),
    BT::OutputPort<std::string>("heard_text")
  };
}


BT::NodeStatus Listen::onStart()
{
  finished_.store(false);
  success_ = false;

  {
    std::lock_guard<std::mutex> lk(mx_);
    heard_text_.clear();
    last_state_.clear();
    last_error_.clear();
    goal_handle_.reset();
  }

  start_timeout_sec_ = getInput<float>("start_timeout_sec").value_or(10.0f);
  const std::string action_name = getInput<std::string>("action_name").value_or("/asr/listen");

  client_ = rclcpp_action::create_client<ListenAction>(node_, action_name);

  if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", ("action:" + action_name).c_str(), "server not available");
    return BT::NodeStatus::FAILURE;
  }

  ListenAction::Goal goal;
  goal.timeout_sec = start_timeout_sec_;

  start_tp_ = std::chrono::steady_clock::now();

  rclcpp_action::Client<ListenAction>::SendGoalOptions options;

  options.goal_response_callback =
    [this](std::shared_ptr<GoalHandleListen> gh)
    {
      if (!gh) {
        std::lock_guard<std::mutex> lk(mx_);
        last_error_ = "goal rejected";
        success_ = false;
        finished_.store(true);
        return;
      }
      std::lock_guard<std::mutex> lk(mx_);
      goal_handle_ = gh;
      last_state_ = "goal accepted";
      BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING", "action goal accepted");
    };

  options.feedback_callback =
    [this](GoalHandleListen::SharedPtr,
           const std::shared_ptr<const ListenAction::Feedback> fb)
    {
      std::lock_guard<std::mutex> lk(mx_);
      last_state_ = fb->state;
      // 디버그용
      BT_LOG_STATUS(node_->get_logger(), name(), "INFO", "feedback", last_state_);
    };

  options.result_callback =
    [this](const GoalHandleListen::WrappedResult& wr)
    {
      std::lock_guard<std::mutex> lk(mx_);

      if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result) {
        success_ = wr.result->success;
        heard_text_ = wr.result->text;
        last_error_.clear();
      } else if (wr.code == rclcpp_action::ResultCode::ABORTED) {
        success_ = false;
        last_error_ = "aborted";
      } else if (wr.code == rclcpp_action::ResultCode::CANCELED) {
        success_ = false;
        last_error_ = "canceled";
      } else {
        success_ = false;
        last_error_ = "unknown result";
      }

      finished_.store(true);
    };

  client_->async_send_goal(goal, options);
  BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING", "action send_goal");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Listen::onRunning()
{
  if (!finished_.load()) {
    return BT::NodeStatus::RUNNING;
  }

  bool ok;
  std::string text;
  std::string err;
  {
    std::lock_guard<std::mutex> lk(mx_);
    ok = success_;
    text = heard_text_;
    err = last_error_;
  }

  if (ok) {
    setOutput("heard_text", text);
    BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS", "heard_text", text);
    return BT::NodeStatus::SUCCESS;
  }

  BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "action result", err);
  return BT::NodeStatus::FAILURE;
}

void Listen::onHalted()
{
  BT_LOG_STATUS(node_->get_logger(), name(), "HALTED", "cancel goal");

  GoalHandleListen::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lk(mx_);
    gh = goal_handle_;
    goal_handle_.reset();
    heard_text_.clear();
    last_state_.clear();
    last_error_.clear();
    success_ = false;
    finished_.store(false);
  }
  if (gh) client_->async_cancel_goal(gh);
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<Listen>("Listen");
}

} // namespace Listen
