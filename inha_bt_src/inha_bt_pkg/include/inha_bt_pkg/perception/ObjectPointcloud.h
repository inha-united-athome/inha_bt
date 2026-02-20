#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <inha_interfaces/action/detection.hpp> 
#include <atomic>
#include <mutex>
#include <string>

namespace ObjectPointcloud
{

class ObjectPointcloud : public BT::StatefulActionNode
{
public:
  ObjectPointcloud(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("camera_id", "right", "camera/arm id (e.g., left/right)"),
      BT::InputPort<std::string>("target_object", "", "target object name (e.g., Sprite, cup)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ✅ msg type 이름 변경
  using ActionT = inha_interfaces::action::Detection;
  using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;

  std::atomic_bool done_{false};
  std::atomic_bool ok_{false};

  std::mutex mtx_;
  GoalHandleT::SharedPtr goal_handle_;

  bool printed_waiting_{false};

  void ensureClient();

  // ✅ action server name 유지 (앞에 / 포함 OK)
  static constexpr const char* kActionName = "/grounded_sam_detection";
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace ObjectPointcloud
