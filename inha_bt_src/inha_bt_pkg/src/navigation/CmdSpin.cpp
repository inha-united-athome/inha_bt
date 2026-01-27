#include "inha_bt_pkg/navigation/CmdSpin.h"

namespace CmdVelRotate
{
CmdSpin::CmdSpin(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus CmdSpin::onStart()
{
    auto topic = getInput<std::string>("topic").value_or("/cmd_vel");
    duration_ms_ = getInput<int>("duration_ms").value_or(9000);
    angular_z_ = getInput<double>("angular_z").value_or(-0.2);

    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    start_time_ = node_->now();
    
    RCLCPP_INFO(node_->get_logger(), "[CmdSpin] START: %.2f rad/s", angular_z_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CmdSpin::onRunning()
{
    auto elapsed = node_->now() - start_time_;
    if (elapsed.seconds() * 1000 >= duration_ms_) {
        stopRobot();
        RCLCPP_INFO(node_->get_logger(), "[CmdSpin] SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::Twist msg;
    msg.angular.z = angular_z_;
    pub_->publish(msg);

    return BT::NodeStatus::RUNNING;
}

void CmdSpin::onHalted()
{
    stopRobot();
}

void CmdSpin::stopRobot()
{
    if (pub_) {
        pub_->publish(geometry_msgs::msg::Twist());
    }
}

// ★ 이 함수가 XML의 "CmdSpin" 문자열과 C++ 클래스를 연결해줍니다!
void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<CmdSpin>("CmdSpin");
}

} // namespace CmdVelRotate