#include "inha_bt_pkg/hri/VlmQuery.h"

#include <chrono>
#include <mutex>
#include <atomic>
#include <sstream>

namespace VlmQuery
{

// ------------------ 로그 헬퍼 ------------------
static inline std::string shorten(const std::string& s, size_t max_len = 120)
{
  if (s.size() <= max_len) return s;
  return s.substr(0, max_len) + "...(" + std::to_string(s.size()) + " chars)";
}

static inline const char* rcToStr(rclcpp_action::ResultCode c)
{
  switch (c) {
    case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:   return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:  return "CANCELED";
    default: return "UNKNOWN";
  }
}

static inline void VLM_LOG(
  const rclcpp::Logger& logger,
  const char* level,  // "I","W","E"
  const std::string& phase,
  const std::string& action_name,
  int mode,
  int timeout_ms,
  const std::string& text,
  const std::string& prompt,
  const std::string& context_json,
  const std::string& extra = "")
{
  std::ostringstream oss;
  oss << "[VlmBT] " << phase
      << " action=" << action_name
      << " mode=" << mode
      << " timeout_ms=" << timeout_ms
      << " text=\"" << shorten(text) << "\""
      << " prompt=\"" << shorten(prompt) << "\""
      << " context_json=\"" << shorten(context_json) << "\"";
  if (!extra.empty()) oss << " | " << extra;

  if (level[0] == 'E') {
    RCLCPP_ERROR(logger, "%s", oss.str().c_str());
  } else if (level[0] == 'W') {
    RCLCPP_WARN(logger, "%s", oss.str().c_str());
  } else {
    RCLCPP_INFO(logger, "%s", oss.str().c_str());
  }
}
// ------------------------------------------------

VlmBT::VlmBT(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList VlmBT::providedPorts()
{
  return {
    BT::InputPort<int>("mode", 1, "1=text, 2=live_image+prompt, 3=context_json"),
    BT::InputPort<std::string>("text", ""),
    BT::InputPort<std::string>("prompt", ""),
    BT::InputPort<std::string>("context_json", ""),
    BT::InputPort<std::string>("action_name", "/vlm/query"),
    BT::InputPort<int>("timeout_ms", 8000, "0=inf"),
    BT::OutputPort<std::string>("generated_text", "")
  };
}

void VlmBT::ensureClient()
{
  // action_name_ 은 onStart에서 읽어온 값 기준
  client_ = rclcpp_action::create_client<VlmAction>(node_, action_name_);
}

BT::NodeStatus VlmBT::onStart()
{
  finished_.store(false);
  success_ = false;

  {
    std::lock_guard<std::mutex> lk(mx_);
    generated_text_.clear();
    error_message_.clear();
    goal_handle_.reset();
  }

  // ---- Inputs ----
  mode_         = getInput<int>("mode").value_or(1);
  text_         = getInput<std::string>("text").value_or("");
  prompt_       = getInput<std::string>("prompt").value_or("");
  context_json_ = getInput<std::string>("context_json").value_or("");
  action_name_  = getInput<std::string>("action_name").value_or("/vlm/query");
  timeout_ms_   = getInput<int>("timeout_ms").value_or(8000);

  VLM_LOG(node_->get_logger(), "I", "START", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_);

  // ---- Client / Server ----
  ensureClient();

  if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
    VLM_LOG(node_->get_logger(), "E", "SERVER_NOT_AVAILABLE", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
            "wait_for_action_server(2s)=false");
    return BT::NodeStatus::FAILURE;
  }

  // ---- Goal ----
  VlmAction::Goal goal;
  goal.mode = mode_;
  goal.text = text_;
  goal.prompt = prompt_;
  goal.context_json = context_json_;
  goal.timeout_ms = timeout_ms_;

  start_tp_ = std::chrono::steady_clock::now();

  rclcpp_action::Client<VlmAction>::SendGoalOptions opt;

  // goal response
  opt.goal_response_callback =
    [this](std::shared_ptr<GoalHandleVlm> gh)
    {
      std::lock_guard<std::mutex> lk(mx_);
      if (!gh) {
        success_ = false;
        error_message_ = "goal rejected";
        finished_.store(true);

        VLM_LOG(node_->get_logger(), "E", "GOAL_REJECTED", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_);
        return;
      }
      goal_handle_ = gh;
      VLM_LOG(node_->get_logger(), "I", "GOAL_ACCEPTED", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_);
    };

  // result
  opt.result_callback =
    [this](const GoalHandleVlm::WrappedResult& wr)
    {
      std::lock_guard<std::mutex> lk(mx_);

      const std::string code_str = rcToStr(wr.code);

      if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result) {
        success_ = wr.result->success;
        generated_text_ = wr.result->generated_text;
        error_message_ = wr.result->error_message;

        if (success_) {
          VLM_LOG(node_->get_logger(), "I", "RESULT_OK", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
                  "code=" + code_str + " out=\"" + shorten(generated_text_) + "\"");
        } else {
          VLM_LOG(node_->get_logger(), "W", "RESULT_FAIL", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
                  "code=" + code_str + " err=\"" + shorten(error_message_) + "\"");
        }

      } else if (wr.code == rclcpp_action::ResultCode::ABORTED) {
        success_ = false;
        error_message_ = "aborted";
        VLM_LOG(node_->get_logger(), "E", "RESULT_ABORTED", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
                "code=" + code_str);

      } else if (wr.code == rclcpp_action::ResultCode::CANCELED) {
        success_ = false;
        error_message_ = "canceled";
        VLM_LOG(node_->get_logger(), "W", "RESULT_CANCELED", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
                "code=" + code_str);

      } else {
        success_ = false;
        error_message_ = "unknown result";
        VLM_LOG(node_->get_logger(), "E", "RESULT_UNKNOWN", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
                "code=" + code_str);
      }

      finished_.store(true);
    };

  // send goal
  VLM_LOG(node_->get_logger(), "I", "SEND_GOAL", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_);
  client_->async_send_goal(goal, opt);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VlmBT::onRunning()
{
  if (!finished_.load()) {
    if (timeout_ms_ > 0) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_tp_).count();

      if (elapsed_ms > timeout_ms_) {
        VLM_LOG(node_->get_logger(), "W", "TIMEOUT", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
                "elapsed_ms=" + std::to_string(elapsed_ms) + " -> cancel goal");

        GoalHandleVlm::SharedPtr gh;
        {
          std::lock_guard<std::mutex> lk(mx_);
          gh = goal_handle_;
          success_ = false;
          error_message_ = "timeout";
        }
        if (gh) {
          client_->async_cancel_goal(gh);
        } else {
          VLM_LOG(node_->get_logger(), "W", "TIMEOUT_NO_GH", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
                  "goal_handle is null (maybe rejected before accept)");
        }

        finished_.store(true);
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  bool ok;
  std::string out, err;
  {
    std::lock_guard<std::mutex> lk(mx_);
    ok = success_;
    out = generated_text_;
    err = error_message_;
  }

  if (ok) {
    setOutput("generated_text", out);
    VLM_LOG(node_->get_logger(), "I", "BT_SUCCESS", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
            "generated_text=\"" + shorten(out) + "\"");
    return BT::NodeStatus::SUCCESS;
  }

  VLM_LOG(node_->get_logger(), "W", "BT_FAILURE", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_,
          "err=\"" + shorten(err) + "\"");
  return BT::NodeStatus::FAILURE;
}

void VlmBT::onHalted()
{
  GoalHandleVlm::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lk(mx_);
    gh = goal_handle_;
    goal_handle_.reset();
    finished_.store(false);
    success_ = false;
    generated_text_.clear();
    error_message_.clear();
  }

  VLM_LOG(node_->get_logger(), "W", "HALT", action_name_, mode_, timeout_ms_, text_, prompt_, context_json_);

  if (gh && client_) {
    client_->async_cancel_goal(gh);
  }
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<VlmBT>("VlmQuery");
}

} // namespace VlmQuery
