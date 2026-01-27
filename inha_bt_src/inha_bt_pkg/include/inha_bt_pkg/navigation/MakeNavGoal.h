#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <string>

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
      // name, default, description (⚠️ 3개 인자 필요)
      BT::InputPort<std::string>("frame", std::string("map"), "target frame"),

      BT::InputPort<double>("x", "goal x"),
      BT::InputPort<double>("y", "goal y"),
      BT::InputPort<double>("z", 0.0, "goal z"),

      BT::InputPort<double>("qx", 0.0, "quat x"),
      BT::InputPort<double>("qy", 0.0, "quat y"),
      BT::InputPort<double>("qz", "quat z"),
      BT::InputPort<double>("qw", "quat w"),

      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "PoseStamped goal")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped pose;

    auto frame = getInput<std::string>("frame");
    auto x  = getInput<double>("x");
    auto y  = getInput<double>("y");
    auto z  = getInput<double>("z");
    auto qx = getInput<double>("qx");
    auto qy = getInput<double>("qy");
    auto qz = getInput<double>("qz");
    auto qw = getInput<double>("qw");

    if (!frame || !x || !y || !z || !qx || !qy || !qz || !qw) {
      return BT::NodeStatus::FAILURE;
    }

    pose.header.frame_id = frame.value();
    pose.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();

    pose.pose.position.x = x.value();
    pose.pose.position.y = y.value();
    pose.pose.position.z = z.value();

    double ox = qx.value();
    double oy = qy.value();
    double oz = qz.value();
    double ow = qw.value();

    // normalize
    const double n = std::sqrt(ox*ox + oy*oy + oz*oz + ow*ow);
    if (n < 1e-12) {
      if (node_) RCLCPP_WARN(node_->get_logger(), "[MakeNavGoal] Quaternion norm too small");
      return BT::NodeStatus::FAILURE;
    }
    ox /= n; oy /= n; oz /= n; ow /= n;

    pose.pose.orientation.x = ox;
    pose.pose.orientation.y = oy;
    pose.pose.orientation.z = oz;
    pose.pose.orientation.w = ow;

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
