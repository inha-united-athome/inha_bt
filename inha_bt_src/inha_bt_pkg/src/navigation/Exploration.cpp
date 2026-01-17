#include "inha_bt_pkg/navigation/Exploration.h"
#include <behaviortree_cpp/bt_factory.h>
#include <chrono> 

namespace Exploration
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

template <typename PublisherT, typename MsgT>
static inline void publish_n_times(
  const rclcpp::Node::SharedPtr& node,
  const PublisherT& pub,
  const MsgT& msg,
  int n = 5,
  int interval_ms = 50,
  const std::string& tag = "")
{
  for (int i = 0; i < n && rclcpp::ok(); ++i) {
    pub->publish(msg);
    if (!tag.empty()) {
      RCLCPP_INFO(node->get_logger(), "[BT][%s] publish #%d/%d (subs=%zu)",
                  tag.c_str(), i + 1, n, pub->get_subscription_count());
    }
    rclcpp::sleep_for(std::chrono::milliseconds(interval_ms));
  }
}


// ----------------- ExplorationAction -----------------
ExplorationAction::ExplorationAction(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  pub_resume_ = node_->create_publisher<std_msgs::msg::Bool>("/explore/resume", 10);

  // 생성 시에는 “현재 진행중”이 아니니 조용히 두거나, 필요하면 READY 1줄만
  // BT_LOG_STATUS(node_->get_logger(), name, "READY", "pub:/explore/resume");
}

BT::PortsList ExplorationAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node")
  };
}

BT::NodeStatus ExplorationAction::onStart()
{
  std_msgs::msg::Bool msg;
  msg.data = true;

  publish_n_times(node_, pub_resume_, msg, 5, 50, std::string(name()) + " /explore/resume(true)");


  // 현재 진행중인 노드 + IO만 출력
  BT_LOG_STATUS(node_->get_logger(), name(), "RUNNING", "pub:/explore/resume(true)");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExplorationAction::onRunning()
{
  // 스팸 방지: RUNNING 반복 로그 X
  return BT::NodeStatus::RUNNING;
}

void ExplorationAction::onHalted()
{
  std_msgs::msg::Bool msg;
  msg.data = false;
  pub_resume_->publish(msg);

  // HALTED는 “실패”가 아니라 부모(Parallel/Sequence)에 의해 중단될 수 있음
  BT_LOG_STATUS(node_->get_logger(), name(), "HALTED", "pub:/explore/resume(false)");
}


// ----------------- ExplorationSaveGoal -----------------
ExplorationSaveGoal::ExplorationSaveGoal(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  pub_save_ = node_->create_publisher<std_msgs::msg::Bool>("/explore/save", 10);
}

BT::PortsList ExplorationSaveGoal::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node")
  };
}

BT::NodeStatus ExplorationSaveGoal::tick()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  pub_save_->publish(msg);

  BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS", "pub:/explore/save(true)");
  return BT::NodeStatus::SUCCESS;
}


// ----------------- ExploreTrigger -----------------
ExploreTrigger::ExploreTrigger(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  pub_init_   = node_->create_publisher<std_msgs::msg::Bool>("/explore/init", 10);
  pub_return_ = node_->create_publisher<std_msgs::msg::Bool>("/explore/return", 10);
}

BT::PortsList ExploreTrigger::providedPorts()
{
  return {
    BT::InputPort<std::string>("cmd", "init"),
    BT::InputPort<rclcpp::Node::SharedPtr>("node")
  };
}

BT::NodeStatus ExploreTrigger::tick()
{
  std::string cmd;
  if (!getInput("cmd", cmd))
  {
    BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE",
                  "pub:/explore/init|/explore/return",
                  "missing input port 'cmd'");
    throw BT::RuntimeError("[ExploreTrigger] Missing input port: cmd");
  }

  std_msgs::msg::Bool msg;
  msg.data = true;

  if (cmd == "init")
  {
    pub_init_->publish(msg);
    BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS", "pub:/explore/init(true)");
    return BT::NodeStatus::SUCCESS;
  }
  if (cmd == "return")
  {
    pub_return_->publish(msg);
    BT_LOG_STATUS(node_->get_logger(), name(), "SUCCESS", "pub:/explore/return(true)");
    return BT::NodeStatus::SUCCESS;
  }

  BT_LOG_STATUS(node_->get_logger(), name(), "FAILURE",
                "pub:/explore/init|/explore/return",
                std::string("invalid cmd='") + cmd + "' (expected init|return)");

  throw BT::RuntimeError("[ExploreTrigger] cmd must be 'init' or 'return'. Got: ", cmd);
}


// ----------------- RegisterNodes -----------------
void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ExplorationAction>("ExplorationAction");
  factory.registerNodeType<ExplorationSaveGoal>("ExplorationSaveGoal");
  factory.registerNodeType<ExploreTrigger>("ExploreTrigger");
}

} // namespace Exploration
