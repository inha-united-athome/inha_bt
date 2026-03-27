#include "inha_bt_pkg/navigation/Approach.h"

#include <chrono>
#include <cctype>
#include <future>

using namespace std::chrono_literals;

namespace Approach
{

namespace
{
std::string trim(const std::string& in)
{
  std::size_t begin = 0;
  while (begin < in.size() && std::isspace(static_cast<unsigned char>(in[begin]))) {
    ++begin;
  }

  std::size_t end = in.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(in[end - 1]))) {
    --end;
  }

  return in.substr(begin, end - begin);
}
}  // namespace

ApproachBT::ApproachBT(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("@node");
  if (!node_) {
    throw BT::RuntimeError("ApproachBT: missing 'node' in blackboard");
  }
}

void ApproachBT::resetState()
{
  goal_sent_.store(false, std::memory_order_relaxed);
  done_.store(false, std::memory_order_relaxed);
  success_.store(false, std::memory_order_relaxed);

  std::lock_guard<std::mutex> lk(mtx_);
  goal_handle_.reset();
  success_message_.clear();
  x_error_ = 0.0;
  y_error_ = 0.0;
  theta_error_ = 0.0;

  (void)setOutput("success_message", std::string(""));
  (void)setOutput("x_error", x_error_);
  (void)setOutput("y_error", y_error_);
  (void)setOutput("theta_error", theta_error_);
}

bool ApproachBT::ensureActionClient(const std::string& action_name)
{
  if (!action_client_ || action_name != action_name_) {
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);
    action_name_ = action_name;
  }
  return action_client_->wait_for_action_server(300ms);
}

bool ApproachBT::ensureServiceClient(const std::string& service_name)
{
  if (!service_client_ || service_name != service_name_) {
    service_client_ = node_->create_client<SrvT>(service_name);
    service_name_ = service_name;
  }
  return service_client_->wait_for_service(300ms);
}

std::vector<std::string> ApproachBT::parseTargets(const std::string& csv, const std::string& fallback)
{
  std::vector<std::string> out;

  if (!csv.empty()) {
    std::size_t start = 0;
    while (start <= csv.size()) {
      const std::size_t comma = csv.find(',', start);
      const std::size_t end = (comma == std::string::npos) ? csv.size() : comma;

      const auto token = trim(csv.substr(start, end - start));
      if (!token.empty()) {
        out.push_back(token);
      }

      if (comma == std::string::npos) {
        break;
      }
      start = comma + 1;
    }
  }

  if (out.empty() && !fallback.empty()) {
    out.push_back(fallback);
  }

  return out;
}

BT::NodeStatus ApproachBT::onStart()
{
  resetState();

  const auto main_target_opt = getInput<std::string>("main_target");
  if (!main_target_opt || main_target_opt.value().empty()) {
    RCLCPP_WARN(node_->get_logger(), "[Approach] missing required input: main_target");
    return BT::NodeStatus::FAILURE;
  }

  const std::string main_target = main_target_opt.value();
  const bool start = getInput<bool>("start").value_or(true);
  const std::string action_name = getInput<std::string>("action_name").value_or("/approach");
  const std::string service_name = getInput<std::string>("service_name").value_or("/detect_object");

  const std::string targets_csv = getInput<std::string>("targets_csv").value_or("");
  const auto targets = parseTargets(targets_csv, main_target);

  if (targets.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[Approach] service target list is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (!ensureServiceClient(service_name)) {
    RCLCPP_WARN(node_->get_logger(), "[Approach] service not available: %s", service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto srv_req = std::make_shared<SrvT::Request>();
  srv_req->start = start;
  srv_req->target = targets;

  auto far = service_client_->async_send_request(srv_req);
  auto srv_future = far.future.share();
  if (srv_future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(), "[Approach] service timeout: %s", service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto srv_resp = srv_future.get();
  if (!srv_resp || !srv_resp->success) {
    RCLCPP_WARN(node_->get_logger(), "[Approach] service failed: %s", service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!ensureActionClient(action_name)) {
    RCLCPP_WARN(node_->get_logger(), "[Approach] action server not available: %s", action_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  ActionT::Goal goal;
  goal.start = start;
  goal.main_target = main_target;

  rclcpp_action::Client<ActionT>::SendGoalOptions options;

  options.goal_response_callback =
    [this](GoalHandle::SharedPtr goal_handle)
    {
      if (!goal_handle) {
        success_.store(false, std::memory_order_relaxed);
        done_.store(true, std::memory_order_relaxed);
        return;
      }

      std::lock_guard<std::mutex> lk(mtx_);
      goal_handle_ = goal_handle;
    };

  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const ActionT::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }

      std::lock_guard<std::mutex> lk(mtx_);
      x_error_ = feedback->x_error;
      y_error_ = feedback->y_error;
      theta_error_ = feedback->theta_error;
    };

  options.result_callback =
    [this](const GoalHandle::WrappedResult& wrapped)
    {
      bool ok = false;
      std::string msg;

      if (wrapped.result) {
        msg = wrapped.result->success_message;
      }

      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result) {
        ok = wrapped.result->success;
      } else if (msg.empty()) {
        msg = std::string("result_code=") + std::to_string(static_cast<int>(wrapped.code));
      }

      {
        std::lock_guard<std::mutex> lk(mtx_);
        success_message_ = msg;
      }

      success_.store(ok, std::memory_order_relaxed);
      done_.store(true, std::memory_order_relaxed);
    };

  (void)action_client_->async_send_goal(goal, options);
  goal_sent_.store(true, std::memory_order_relaxed);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachBT::onRunning()
{
  if (!goal_sent_.load(std::memory_order_relaxed)) {
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    setOutput("x_error", x_error_);
    setOutput("y_error", y_error_);
    setOutput("theta_error", theta_error_);
  }

  if (!done_.load(std::memory_order_relaxed)) {
    return BT::NodeStatus::RUNNING;
  }

  std::string success_message;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    success_message = success_message_;
  }

  setOutput("success_message", success_message);

  return success_.load(std::memory_order_relaxed) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
}

void ApproachBT::onHalted()
{
  GoalHandle::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_handle = goal_handle_;
  }

  if (action_client_ && goal_handle) {
    (void)action_client_->async_cancel_goal(goal_handle);
  }

  resetState();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ApproachBT>("Approach");
}

}  // namespace Approach
