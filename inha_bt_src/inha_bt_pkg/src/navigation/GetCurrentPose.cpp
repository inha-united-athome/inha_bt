#include "inha_bt_pkg/navigation/GetCurrentPose.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <string>
using namespace std::chrono_literals;

namespace
{
// ---- 공통: 요약 로그 헬퍼 ----
static inline void BT_LOG_STATUS(
  const rclcpp::Logger& logger,
  const std::string& node_name,
  const char* status,
  const char* io,
  const std::string& reason = "")
{
  if (!reason.empty()) {
    RCLCPP_WARN(logger, "[BT] %s : %s  (%s)  reason=%s",
                node_name.c_str(), status, io, reason.c_str());
  } else {
    RCLCPP_INFO(logger, "[BT] %s : %s  (%s)",
                node_name.c_str(), status, io);
  }
}
} // namespace

namespace GetCurrentPose
{

GetCurrentPose::GetCurrentPose(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("GetCurrentPose: missing 'node' in blackboard");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  BT_LOG_STATUS(node_->get_logger(), this->name(), "INIT", "in(bb)", "tf listener ready");
}

BT::NodeStatus GetCurrentPose::tick()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    // map <- base_link
    tf = tf_buffer_->lookupTransform(kTargetFrame, kSourceFrame, tf2::TimePointZero, 200ms);
  } catch (const std::exception& e) {
    BT_LOG_STATUS(node_->get_logger(), this->name(), "FAILURE", "in(tf)",
                  std::string(kTargetFrame) + "<-" + kSourceFrame + "  " + e.what());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = tf.header;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation = tf.transform.rotation;

   // yaw 추출 (rad)
  const double yaw = tf2::getYaw(pose.pose.orientation);

  setOutput("pose", pose);

  // 보기 좋게 한 줄 로그
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << kTargetFrame << "<-" << kSourceFrame
      << "  x=" << pose.pose.position.x
      << " y=" << pose.pose.position.y
      << " z=" << pose.pose.position.z
      << " yaw=" << yaw;



  BT_LOG_STATUS(node_->get_logger(), this->name(), "SUCCESS", "out(pose)", oss.str());


  return BT::NodeStatus::SUCCESS;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<GetCurrentPose>("GetCurrentPose");
}

} // namespace GetCurrentPose
