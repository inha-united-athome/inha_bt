#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace WaitBoolTopicBT
{

class WaitBoolTopic : public BT::StatefulActionNode
{
public:
  WaitBoolTopic(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw BT::RuntimeError("WaitBoolTopic: missing 'node' in blackboard");
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/bt/trigger", "std_msgs/Bool trigger topic"),
      BT::InputPort<bool>("reset_on_start", true, "Reset trigger flag onStart"),
      BT::InputPort<bool>("success_on_true", true, "If true->SUCCESS, else false->SUCCESS"),
    };
  }

  BT::NodeStatus onStart() override
  {
    topic_ = getInput<std::string>("topic").value_or("/bt/trigger");
    const bool reset_on_start = getInput<bool>("reset_on_start").value_or(true);
    success_on_true_ = getInput<bool>("success_on_true").value_or(true);

    {
      std::lock_guard<std::mutex> lk(mx_);
      if (reset_on_start) triggered_ = false;
      last_msg_ = false;
    }

    sub_.reset();
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(mx_);
        last_msg_ = msg->data;
        // 원하는 값(true or false)이 오면 트리거
        if ((success_on_true_ && msg->data) || (!success_on_true_ && !msg->data)) {
          triggered_ = true;
        }
      });

    RCLCPP_INFO(node_->get_logger(),
                "[WaitBoolTopic] waiting topic=%s (success_on_%s)",
                topic_.c_str(), success_on_true_ ? "true" : "false");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lk(mx_);
    return triggered_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    sub_.reset();
    std::lock_guard<std::mutex> lk(mx_);
    triggered_ = false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  std::string topic_;
  std::mutex mx_;
  bool triggered_{false};
  bool last_msg_{false};
  bool success_on_true_{true};
};

// ✅ 너가 원하는 패턴: header에서 바로 등록 함수 제공
inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<WaitBoolTopic>("WaitBoolTopic");
}

} // namespace WaitBoolTopicBT
