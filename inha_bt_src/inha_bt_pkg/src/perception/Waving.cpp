#include "inha_bt_pkg/perception/Waving.h"
#include <behaviortree_cpp/bt_factory.h>

namespace Waving
{

Waving::Waving(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 1) /human/states : HAND_UP_LEFT or HAND_UP_RIGHT 포함이면 detected=true
  sub_states_ = node_->create_subscription<std_msgs::msg::String>(
    "/human/states", 10,
    [this](std_msgs::msg::String::SharedPtr msg)
    {
      if (!active_.load(std::memory_order_relaxed)) return;

      const auto& s = msg->data;
      if (s.find("HAND_UP_LEFT") != std::string::npos ||
          s.find("HAND_UP_RIGHT") != std::string::npos)
      {
        detected_.store(true, std::memory_order_relaxed);
      }
    });

  // 2) /human/hand_up_goal : PoseStamped 저장
  sub_goal_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/human/hand_up_goal", 10,
    [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_goal_ = *msg;
      pose_ready_.store(true, std::memory_order_relaxed);
    });

  // 생성 시에는 “구독 정보만” 짧게
  RCLCPP_INFO(node_->get_logger(),
              "[%s] : RUNNING (sub=/human/states, /human/hand_up_goal)",
              this->name().c_str());
}

BT::PortsList Waving::providedPorts()
{
  // BT XML에서 <Waving waving_pose_world="{w_pose}"/> 이거랑 맞춰야 함
  return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("waving_pose_world") };
}

BT::NodeStatus Waving::onStart()
{
  active_.store(true, std::memory_order_relaxed);
  detected_.store(false, std::memory_order_relaxed);
  pose_ready_.store(false, std::memory_order_relaxed);
  printed_waiting_ = false;

  // 시작 로그는 1줄
  RCLCPP_INFO(node_->get_logger(),
              "[%s] : RUNNING (waiting HAND_UP + goal_pose)",
              this->name().c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Waving::onRunning()
{
  // 아직 손 들기 감지 안됨
  if (!detected_.load(std::memory_order_relaxed))
  {
    if (!printed_waiting_) {
      printed_waiting_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[%s] : RUNNING (reason=no HAND_UP)",
                  this->name().c_str());
    }
    return BT::NodeStatus::RUNNING;
  }

  // 손 들기는 감지됐는데 goal pose가 아직 안옴
  if (!pose_ready_.load(std::memory_order_relaxed))
  {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] : RUNNING (reason=HAND_UP but no /human/hand_up_goal yet)",
                this->name().c_str());
    return BT::NodeStatus::RUNNING;
  }

  // SUCCESS: output에 pose 넣기 (frame_id는 무조건 map)
  geometry_msgs::msg::PoseStamped out;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    out = last_goal_;
  }
  out.header.frame_id = "map"; // 강제

  setOutput("waving_pose_world", out);

  active_.store(false, std::memory_order_relaxed);

  RCLCPP_INFO(node_->get_logger(),
              "[%s] : SUCCESS",
              this->name().c_str());

  return BT::NodeStatus::SUCCESS;
}

void Waving::onHalted()
{
  active_.store(false, std::memory_order_relaxed);
  RCLCPP_WARN(node_->get_logger(),
              "[%s] : FAILURE (reason=halted)",
              this->name().c_str());
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<Waving>("Waving");
}

} // namespace Waving
