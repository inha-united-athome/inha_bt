#include "inha_bt_pkg/perception/ObjectPointcloud.h"

#include <chrono>
#include <string>
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

namespace ObjectPointcloud
{

ObjectPointcloud::ObjectPointcloud(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("ObjectPointcloud: missing 'node' in blackboard");
  }
  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "got node from blackboard");
}

void ObjectPointcloud::ensureClient()
{
  if (client_) return;
  client_ = rclcpp_action::create_client<ActionT>(node_, kActionName);
  BT_LOG_STATUS(node_->get_logger(), this->name(), "READY", "out(action)",
                std::string("client created: ") + kActionName);
}

BT::NodeStatus ObjectPointcloud::onStart()
{
  ensureClient();

  printed_waiting_ = false;
  done_.store(false);
  ok_.store(false);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_handle_.reset();
  }

  // ✅ 포트명 변경: object -> target_object
  const std::string camera_id      = getInput<std::string>("camera_id").value_or("right");
  const std::string target_object  = getInput<std::string>("target_object").value_or("");

  if (target_object.empty()) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(port)", "target_object is empty");
    return BT::NodeStatus::FAILURE;
  }

  BT_LOG_STATUS(node_->get_logger(), this->name(), "START", "out(action)",
                "send goal camera_id=" + camera_id + ", target_object=" + target_object);

  if (!client_->wait_for_action_server(300ms)) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "out(action)",
                  std::string("action server not available: ") + kActionName);
    return BT::NodeStatus::FAILURE;
  }

  ActionT::Goal goal;
  // ✅ Goal 필드명 CLI와 동일하게
  goal.camera_id = camera_id;
  goal.target_object = target_object;

  rclcpp_action::Client<ActionT>::SendGoalOptions opt;

  opt.goal_response_callback =
    [this](GoalHandleT::SharedPtr gh)
    {
      if (!gh) {
        ok_.store(false);
        done_.store(true);
        BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(goal_response)", "goal rejected");
        return;
      }
      {
        std::lock_guard<std::mutex> lk(mtx_);
        goal_handle_ = gh;
      }
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "in(goal_response)", "goal accepted");
    };

  opt.feedback_callback =
    [this](GoalHandleT::SharedPtr,
           const std::shared_ptr<const ActionT::Feedback> fb)
    {
      (void)fb;
    };

  opt.result_callback =
    [this](const GoalHandleT::WrappedResult& res)
    {
      bool ok = false;
      std::string reason;

      if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
        ok = false;
        reason = "result_code != SUCCEEDED";
      } else if (res.result) {
        ok = res.result->success;
        if (!ok) {
          reason = res.result->error_message;
        }
      } else {
        ok = false;
        reason = "null result";
      }

      ok_.store(ok);
      done_.store(true);

      if (ok) {
        BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "in(result)", "success=true");
      } else {
        BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(result)",
                      reason.empty() ? "success=false" : reason);
      }
    };

  (void)client_->async_send_goal(goal, opt);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ObjectPointcloud::onRunning()
{
  if (!done_.load()) {
    if (!printed_waiting_) {
      printed_waiting_ = true;
      BT_LOG_STATUS(node_->get_logger(), this->name(), "RUNNING", "wait(result)");
    }
    return BT::NodeStatus::RUNNING;
  }

  return ok_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void ObjectPointcloud::onHalted()
{
  GoalHandleT::SharedPtr gh;
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
  factory.registerNodeType<ObjectPointcloud>("ObjectPointcloud");
}

} // namespace ObjectPointcloud
