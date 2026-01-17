#include "inha_bt_pkg/hri/ExtractWord.h"
#include "behaviortree_cpp/bt_factory.h"

#include <nlohmann/json.hpp>

namespace ExtractWordBT
{
using json = nlohmann::json;
using BT::NodeStatus;

ExtractWord::ExtractWord(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList ExtractWord::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node"),
    BT::InputPort<std::string>("heard_text", "Original heard text"),
    BT::InputPort<std::string>("prompt", "Instruction prompt (NOT including heard_text)"),

    // 1이면 extracted_word, 2면 person_name + favorite_drink
    BT::InputPort<int>("word_count", 1, "1 or 2"),

    // word_count==1일 때 result_json에서 어떤 키를 뽑을지
    // auto: name 우선, 없으면 favorite_drink, 없으면 generated_text fallback
    BT::InputPort<std::string>("pick", "auto", "auto|name|favorite_drink"),

    // Python 서버: mode=45가 name/drink JSON을 준다는 전제
    BT::InputPort<int>("vlm_mode", 45, "VLM mode (default:45)"),

    BT::InputPort<std::string>("action_name", "/vlm/query", "VLM action name"),
    BT::InputPort<int>("timeout_ms", 8000, "Timeout(ms), 0=inf"),

    // outputs
    BT::OutputPort<std::string>("extracted_word", "Extracted single word/phrase"),
    BT::OutputPort<std::string>("person_name", "Extracted person name"),
    BT::OutputPort<std::string>("favorite_drink", "Extracted favorite drink"),
    BT::OutputPort<std::string>("result_json", "Raw result_json for debugging")
  };
}

NodeStatus ExtractWord::onStart()
{
  finished_.store(false);
  success_ = false;

  {
    std::lock_guard<std::mutex> lk(mx_);
    last_error_.clear();
    generated_text_.clear();
    result_json_.clear();
    goal_handle_.reset();
  }

  // inputs
  heard_text_  = getInput<std::string>("heard_text").value_or("");
  prompt_      = getInput<std::string>("prompt").value_or("");
  word_count_  = getInput<int>("word_count").value_or(1);
  pick_        = getInput<std::string>("pick").value_or("auto");
  vlm_mode_    = getInput<int>("vlm_mode").value_or(45);
  action_name_ = getInput<std::string>("action_name").value_or("/vlm/query");
  timeout_ms_  = getInput<int>("timeout_ms").value_or(8000);

  // client (onStart에서만 생성)
  client_ = rclcpp_action::create_client<VLM>(node_, action_name_);

  if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "[ExtractWord] Action server not available: %s",
                 action_name_.c_str());
    return NodeStatus::FAILURE;
  }

  // goal 구성: heard_text는 context_json, prompt는 instruction
  VLM::Goal goal;
  goal.mode = vlm_mode_;
  goal.timeout_ms = timeout_ms_;

  json ctx;
  ctx["heard_text"] = heard_text_;
  goal.context_json = ctx.dump();
  goal.prompt = prompt_;

  RCLCPP_INFO(node_->get_logger(),
    "[ExtractWord] mode=%d word_count=%d pick=%s action=%s timeout_ms=%d",
    vlm_mode_, word_count_, pick_.c_str(), action_name_.c_str(), timeout_ms_);

  start_tp_ = std::chrono::steady_clock::now();

  rclcpp_action::Client<VLM>::SendGoalOptions options;

  options.goal_response_callback =
    [this](std::shared_ptr<GoalHandleVLM> gh)
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
    };

  options.result_callback =
    [this](const GoalHandleVLM::WrappedResult& wr)
    {
      std::lock_guard<std::mutex> lk(mx_);

      // result가 있으면 최대한 읽는다 (ABORTED여도 서버가 error_message 넣어줄 수 있음)
      if (wr.result) {
        success_        = wr.result->success;
        generated_text_ = wr.result->generated_text;
        result_json_    = wr.result->result_json;
        last_error_     = wr.result->error_message;
      }

      if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
        // success_는 wr.result->success가 최종
      } else if (wr.code == rclcpp_action::ResultCode::ABORTED) {
        success_ = false;
        if (last_error_.empty()) last_error_ = "aborted";
      } else if (wr.code == rclcpp_action::ResultCode::CANCELED) {
        success_ = false;
        if (last_error_.empty()) last_error_ = "canceled";
      } else {
        success_ = false;
        if (last_error_.empty()) last_error_ = "unknown result";
      }

      finished_.store(true);
    };

  client_->async_send_goal(goal, options);
  return NodeStatus::RUNNING;
}

NodeStatus ExtractWord::onRunning()
{
  if (!finished_.load()) {
    // timeout guard
    if (timeout_ms_ > 0) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_tp_).count();

      if (elapsed_ms > timeout_ms_) {
        RCLCPP_WARN(node_->get_logger(), "[ExtractWord] timeout -> cancel goal");

        GoalHandleVLM::SharedPtr gh;
        {
          std::lock_guard<std::mutex> lk(mx_);
          gh = goal_handle_;
          last_error_ = "timeout";
          success_ = false;
        }
        if (gh) client_->async_cancel_goal(gh);

        finished_.store(true);
        return NodeStatus::FAILURE;
      }
    }
    return NodeStatus::RUNNING;
  }

  bool ok = false;
  std::string gen, rjson, err;
  {
    std::lock_guard<std::mutex> lk(mx_);
    ok    = success_;
    gen   = generated_text_;
    rjson = result_json_;
    err   = last_error_;
  }

  // 디버깅용 raw json 출력
  setOutput("result_json", rjson);

  if (!ok) {
    RCLCPP_WARN(node_->get_logger(), "[ExtractWord] failed: %s", err.c_str());
    return NodeStatus::FAILURE;
  }

  // 성공이면 result_json 파싱해서 값 뽑기
  std::string name, drink;
  if (!rjson.empty()) {
    json j = json::parse(rjson, nullptr, false);
    if (!j.is_discarded()) {
      name  = j.value("name", "");
      drink = j.value("favorite_drink", "");
    }
  }

  if (word_count_ <= 1) {
    std::string out;
    if (pick_ == "name") out = name;
    else if (pick_ == "favorite_drink") out = drink;
    else { // auto
      out = !name.empty() ? name : (!drink.empty() ? drink : gen);
    }

    if (out.empty()) {
      RCLCPP_WARN(node_->get_logger(), "[ExtractWord] success but empty extracted_word");
      return NodeStatus::FAILURE;
    }

    setOutput("extracted_word", out);
    return NodeStatus::SUCCESS;
  }

  // word_count==2
  // 최소 name은 있어야 성공으로 처리 (drink는 비어도 허용)
  if (name.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[ExtractWord] success but empty person_name");
    return NodeStatus::FAILURE;
  }

  setOutput("person_name", name);
  setOutput("favorite_drink", drink);
  return NodeStatus::SUCCESS;
}

void ExtractWord::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[ExtractWord] HALTED -> cancel goal");

  GoalHandleVLM::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lk(mx_);
    gh = goal_handle_;
    goal_handle_.reset();
    success_ = false;
    finished_.store(false);
    last_error_.clear();
    generated_text_.clear();
    result_json_.clear();
  }
  if (gh) client_->async_cancel_goal(gh);
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ExtractWord>("ExtractWord");
}

}  // namespace ExtractWordBT
