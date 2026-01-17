#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <mutex>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inha_interfaces/action/speak.hpp>
#include <behaviortree_cpp/bt_factory.h>

namespace Speak
{

using SpeakAction = inha_interfaces::action::Speak;
using GoalHandleSpeak = rclcpp_action::ClientGoalHandle<SpeakAction>;

class Speak : public BT::StatefulActionNode
{
public:
    // Constructor
    Speak(const std::string& name, const BT::NodeConfig& config);

    // Provided ports for BehaviorTree
    static BT::PortsList providedPorts();

    // BT interface overrides
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // ROS2 node handle
    rclcpp::Node::SharedPtr node_;

    // Action client
    rclcpp_action::Client<SpeakAction>::SharedPtr client_;
    GoalHandleSpeak::SharedPtr goal_handle_;

    // Input ports
    std::string action_name_;

    // Internal state
    std::atomic<bool> goal_sent_;
    std::atomic<bool> done_;
    std::atomic<bool> success_;

    std::mutex mtx_;

    // Helper functions
    void resetState();
    bool ensureClient();
    void sendGoal(const std::string &text);
};

// Factory registration
void RegisterNodes(BT::BehaviorTreeFactory &factory);

} // namespace Speak
