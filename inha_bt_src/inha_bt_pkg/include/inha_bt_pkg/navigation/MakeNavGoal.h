#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>   // ✅ 추가

namespace MakeNavGoal
{

class MakeNavGoal : public BT::SyncActionNode
{
public:
  MakeNavGoal(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("frame", "map"),
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("yaw"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped pose;

    auto frame = getInput<std::string>("frame");
    auto x = getInput<double>("x");
    auto y = getInput<double>("y");
    auto yaw = getInput<double>("yaw");
    if (!frame || !x || !y || !yaw) return BT::NodeStatus::FAILURE;

    pose.header.frame_id = frame.value();
    pose.header.stamp = node_->now();
    pose.pose.position.x = x.value();
    pose.pose.position.y = y.value();
    pose.pose.position.z = 0.0;

    // ✅ yaw -> quaternion (roll=pitch=0)
    const double half = yaw.value() * 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(half);
    pose.pose.orientation.w = std::cos(half);

    setOutput("goal", pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<MakeNavGoal>("MakeNavGoal");
}

} // namespace MakeNavGoal
