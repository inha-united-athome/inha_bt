#include "inha_bt_pkg/hri/Speak.h"
#include <chrono>

namespace Speak
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

Speak::Speak(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      goal_sent_(false), done_(false), success_(false)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList Speak::providedPorts()
{
    return {
        BT::InputPort<std::string>("text"),
        BT::InputPort<std::string>("action_name", "speak")
    };
}

void Speak::resetState()
{
    goal_sent_ = false;
    done_ = false;
    success_ = false;

    // ✅ goal_handle_는 콜백/halting과 경쟁하니 mutex로 보호
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal_handle_.reset();
    }
}

bool Speak::ensureClient()
{
    if (!client_)
    {
        client_ = rclcpp_action::create_client<inha_interfaces::action::Speak>(node_, action_name_);
    }
    if (!client_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
        BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "action server not available");
        return false;
    }
    return true;
}

void Speak::sendGoal(const std::string &text)
{
    inha_interfaces::action::Speak::Goal goal_msg;
    goal_msg.text = text;

    rclcpp_action::Client<inha_interfaces::action::Speak>::SendGoalOptions options;

    options.goal_response_callback =
        [this](const GoalHandleSpeak::SharedPtr handle)
        {
            // ✅ goal_handle_ 저장은 mutex
            {
              std::lock_guard<std::mutex> lk(mtx_);
              goal_handle_ = handle;
            }

            if (!handle)
            {
                done_ = true;
                success_ = false;
                BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "goal rejected");
            }
        };

    options.result_callback =
        [this](const GoalHandleSpeak::WrappedResult &result)
        {
            done_ = true;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                success_ = true;
                BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS", "Speak finished");
            }
            else
            {
                success_ = false;
                BT_LOG_STATUS(
                  node_->get_logger(), name(), "FAILURE", "Speak finished",
                  "result code " + std::to_string(static_cast<int>(result.code)));
            }
        };

    client_->async_send_goal(goal_msg, options);
    goal_sent_ = true;

    BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING", "goal sent", text);
}

BT::NodeStatus Speak::onStart()
{
    resetState();
    action_name_ = getInput<std::string>("action_name").value_or("/speak");

    auto text_any = getInput<std::string>("text");
    if (!text_any)
    {
        BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "missing input port 'text'");
        return BT::NodeStatus::FAILURE;
    }

    if (!ensureClient())
    {
        return BT::NodeStatus::FAILURE;
    }

    sendGoal(text_any.value());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Speak::onRunning()
{
    if (!goal_sent_)
    {
        BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE", "goal not sent");
        return BT::NodeStatus::FAILURE;
    }

    if (!done_)
    {
        return BT::NodeStatus::RUNNING;
    }

    return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void Speak::onHalted()
{
    // ✅ goal_handle_ 읽기도 mutex로 안전하게 복사 후 사용
    GoalHandleSpeak::SharedPtr handle_copy;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      handle_copy = goal_handle_;
    }

    if (client_ && handle_copy)
    {
        client_->async_cancel_goal(handle_copy);
        BT_LOG_STATUS(node_->get_logger(), name(), "HALTED", "goal canceled");
    }
    resetState();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  // 일단 너 원하는대로 1-arg 유지
  factory.registerNodeType<Speak>("Speak");
  // 만약 여기서 또 overload 에러 나면 그때만:
  // factory.registerNodeType<Speak>("Speak", Speak::providedPorts());
}

} // namespace Speak
