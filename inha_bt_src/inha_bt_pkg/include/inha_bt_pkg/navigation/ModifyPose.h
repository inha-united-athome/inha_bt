#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace ModifyPose
{

class ModifyPose : public BT::SyncActionNode
{
public:
  ModifyPose(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("in_pose"),

      // ✅ position delta
      BT::InputPort<double>("dx", 0.0, "delta x"),
      BT::InputPort<double>("dy", 0.0, "delta y"),
      BT::InputPort<double>("dz", 0.0, "delta z"),

      // ✅ yaw delta (deg)
      BT::InputPort<double>("dyaw_deg", 0.0, "delta yaw in degrees"),

      BT::OutputPort<geometry_msgs::msg::PoseStamped>("out_pose")
    };
  }

  BT::NodeStatus tick() override
  {
    auto in = getInput<geometry_msgs::msg::PoseStamped>("in_pose");
    auto dx = getInput<double>("dx");
    auto dy = getInput<double>("dy");
    auto dz = getInput<double>("dz");
    auto dyaw_deg = getInput<double>("dyaw_deg");

    if (!in || !dx || !dy || !dz || !dyaw_deg) {
      return BT::NodeStatus::FAILURE;
    }

    auto out = in.value();

    // 1) position += delta
    out.pose.position.x += dx.value();
    out.pose.position.y += dy.value();
    out.pose.position.z += dz.value();

    // 2) yaw += delta (keep roll/pitch)
    tf2::Quaternion q_in;
    tf2::fromMsg(out.pose.orientation, q_in);

    double r, p, y;
    tf2::Matrix3x3(q_in).getRPY(r, p, y);

    const double dyaw = dyaw_deg.value() * M_PI / 180.0;

    tf2::Quaternion q_out;
    q_out.setRPY(r, p, y + dyaw);
    q_out.normalize();

    out.pose.orientation = tf2::toMsg(q_out);

    setOutput("out_pose", out);
    return BT::NodeStatus::SUCCESS;
  }
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ModifyPose>("ModifyPose");
}

} // namespace ModifyPose
