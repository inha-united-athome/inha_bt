#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <mutex>
#include <string>

namespace IsGoalNearby
{

class IsGoalNearby : public BT::ConditionNode
{
public:
  IsGoalNearby(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("@node");
    if (!node_) {
      throw BT::RuntimeError("IsGoalNearby: missing 'node' in blackboard");
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target goal pose"),
      BT::InputPort<double>("threshold", 0.5, "Distance threshold in meters"),
      BT::InputPort<std::string>("amcl_topic", "/amcl_pose", "AMCL pose topic"),
      BT::OutputPort<double>("distance", "Computed distance to goal in meters")
    };
  }

  BT::NodeStatus tick() override
  {
    ensureSubscription();

    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[IsGoalNearby] missing input port: goal");
      return BT::NodeStatus::FAILURE;
    }

    const double threshold = getInput<double>("threshold").value_or(0.5);
    if (!std::isfinite(threshold) || threshold < 0.0) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[IsGoalNearby] invalid threshold: %.3f", threshold);
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!has_amcl_pose_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[IsGoalNearby] waiting first AMCL pose from topic: %s",
          amcl_topic_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      amcl_pose = latest_amcl_pose_;
    }

    if (!goal->header.frame_id.empty() &&
        !amcl_pose.header.frame_id.empty() &&
        goal->header.frame_id != amcl_pose.header.frame_id)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[IsGoalNearby] frame mismatch: goal=%s amcl=%s",
        goal->header.frame_id.c_str(), amcl_pose.header.frame_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const double dx = goal->pose.position.x - amcl_pose.pose.pose.position.x;
    const double dy = goal->pose.position.y - amcl_pose.pose.pose.position.y;

    const double distance = std::hypot(dx, dy);
    setOutput("distance", distance);

    return (distance <= threshold) ? BT::NodeStatus::SUCCESS
                                   : BT::NodeStatus::FAILURE;
  }

private:
  void ensureSubscription()
  {
    const std::string topic = getInput<std::string>("amcl_topic").value_or("/amcl_pose");
    if (sub_ && topic == amcl_topic_) {
      return;
    }

    amcl_topic_ = topic;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      has_amcl_pose_ = false;
    }

    sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_amcl_pose_ = *msg;
        has_amcl_pose_ = true;
      });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;

  std::mutex mtx_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_amcl_pose_;
  bool has_amcl_pose_{false};
  std::string amcl_topic_{"/amcl_pose"};
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<IsGoalNearby>("IsGoalNearby");
}

} // namespace IsGoalNearby
