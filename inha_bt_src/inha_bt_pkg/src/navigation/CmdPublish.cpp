#include "inha_bt_pkg/navigation/CmdPublish.h"

namespace CmdPublish
{

CmdPublish::CmdPublish(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus CmdPublish::onStart()
{
  auto topic = getInput<std::string>("topic").value_or("/cmd_vel");
  duration_sec_ = getInput<double>("duration_sec").value_or(1.0);

  // cmd 값 읽기 (없으면 기본 0)
  cmd_.linear.x  = getInput<double>("lin_x").value_or(0.0);
  cmd_.linear.y  = getInput<double>("lin_y").value_or(0.0);
  cmd_.linear.z  = getInput<double>("lin_z").value_or(0.0);
  cmd_.angular.x = getInput<double>("ang_x").value_or(0.0);
  cmd_.angular.y = getInput<double>("ang_y").value_or(0.0);
  cmd_.angular.z = getInput<double>("ang_z").value_or(0.0);

  pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
  start_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "[CmdPublish] START topic=%s duration=%.2fs lin(%.2f %.2f %.2f) ang(%.2f %.2f %.2f)",
    topic.c_str(), duration_sec_,
    cmd_.linear.x, cmd_.linear.y, cmd_.linear.z,
    cmd_.angular.x, cmd_.angular.y, cmd_.angular.z
  );

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CmdPublish::onRunning()
{
  const auto elapsed = node_->now() - start_time_;
  if (elapsed.seconds() >= duration_sec_) {
    stopRobot();
    RCLCPP_INFO(node_->get_logger(), "[CmdPublish] SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  if (pub_) pub_->publish(cmd_);
  return BT::NodeStatus::RUNNING;
}

void CmdPublish::onHalted()
{
  stopRobot();
}

void CmdPublish::stopRobot()
{
  if (pub_) {
    pub_->publish(geometry_msgs::msg::Twist());  // all zeros
  }
}

// XML의 "CmdPublish"와 C++ 클래스를 연결
void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<CmdPublish>("CmdPublish");
}

} // namespace CmdPublish
