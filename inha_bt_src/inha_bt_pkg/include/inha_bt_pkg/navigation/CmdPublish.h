#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace CmdPublish
{

class CmdPublish : public BT::StatefulActionNode
{
public:
  CmdPublish(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      // ✅ seconds로 받기
      BT::InputPort<double>("duration_sec", 1.0, "publish duration (sec)"),
      BT::InputPort<std::string>("topic", "/cmd_vel", "topic name"),

      // ✅ linear
      BT::InputPort<double>("lin_x", 0.0, "linear x"),
      BT::InputPort<double>("lin_y", 0.0, "linear y"),
      BT::InputPort<double>("lin_z", 0.0, "linear z"),

      // ✅ angular
      BT::InputPort<double>("ang_x", 0.0, "angular x"),
      BT::InputPort<double>("ang_y", 0.0, "angular y"),
      BT::InputPort<double>("ang_z", 0.0, "angular z"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void stopRobot();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  rclcpp::Time start_time_;
  double duration_sec_{1.0};

  geometry_msgs::msg::Twist cmd_;
};

// bt_main.cpp에서 등록할 때 쓰는 함수
void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace CmdPublish
