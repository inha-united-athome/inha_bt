#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace GetCurrentPose
{

class GetCurrentPose : public BT::SyncActionNode
{
public:
  GetCurrentPose(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Current pose (map->base_link)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ✅ 하드코딩
  static constexpr const char* kTargetFrame = "map";
  static constexpr const char* kSourceFrame = "base";
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace GetCurrentPose
