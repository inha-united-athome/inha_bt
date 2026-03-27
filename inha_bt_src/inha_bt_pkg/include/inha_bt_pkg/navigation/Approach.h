#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <inha_interfaces/action/approach.hpp>
#include <inha_interfaces/srv/detect_object.hpp>

namespace Approach
{

class ApproachBT : public BT::StatefulActionNode
{
public:
  using ActionT = inha_interfaces::action::Approach;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using SrvT = inha_interfaces::srv::DetectObject;

  ApproachBT(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("main_target", "action goal main target"),
      BT::InputPort<std::string>("targets_csv", "", "optional comma-separated targets for service request"),
      BT::InputPort<bool>("start", true, "start flag used for both service and action"),
      BT::InputPort<std::string>("action_name", "/approach", "Approach action server name"),
      BT::InputPort<std::string>("service_name", "/detect_object", "Detect_object service name"),
      BT::OutputPort<std::string>("success_message", "action success message"),
      BT::OutputPort<double>("x_error", "latest x error from feedback"),
      BT::OutputPort<double>("y_error", "latest y error from feedback"),
      BT::OutputPort<double>("theta_error", "latest theta error from feedback")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void resetState();
  bool ensureActionClient(const std::string& action_name);
  bool ensureServiceClient(const std::string& service_name);

  static std::vector<std::string> parseTargets(const std::string& csv, const std::string& fallback);

  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  rclcpp::Client<SrvT>::SharedPtr service_client_;

  std::string action_name_;
  std::string service_name_;

  std::mutex mtx_;
  GoalHandle::SharedPtr goal_handle_;

  std::atomic_bool goal_sent_{false};
  std::atomic_bool done_{false};
  std::atomic_bool success_{false};

  std::string success_message_;
  double x_error_{0.0};
  double y_error_{0.0};
  double theta_error_{0.0};
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace Approach
