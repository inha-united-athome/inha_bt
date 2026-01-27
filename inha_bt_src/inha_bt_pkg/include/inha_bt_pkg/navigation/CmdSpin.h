#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"  // Factory 포함 확인
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace CmdVelRotate
{
class CmdSpin : public BT::StatefulActionNode
{
public:
    CmdSpin(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("duration_ms", 9000, "회전 시간 (ms)"),
            BT::InputPort<double>("angular_z", -0.2, "회전 속도 (rad/s)"),
            BT::InputPort<std::string>("topic", "/cmd_vel", "토픽 이름")
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
    int duration_ms_;
    double angular_z_;
};

// ★ 이 선언이 반드시 있어야 bt_main.cpp에서 인식합니다!
void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace CmdVelRotate