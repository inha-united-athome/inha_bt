#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
// util
#include "inha_bt_pkg/utils/ServiceTrigger.h"

// nav
#include "inha_bt_pkg/navigation/Exploration.h"
#include "inha_bt_pkg/navigation/WaitGoalReached.h"
#include "inha_bt_pkg/navigation/GoToPose.h"
#include "inha_bt_pkg/navigation/MakeNavGoal.h"
#include "inha_bt_pkg/navigation/FollowHuman.h"
#include "inha_bt_pkg/navigation/CmdPublish.h"
#include "inha_bt_pkg/navigation/WaitBoolTopic.hpp"
#include "inha_bt_pkg/navigation/GetCurrentPose.h"
#include "inha_bt_pkg/navigation/ModifyPose.h"
#include "inha_bt_pkg/navigation/GoToPrecise.h"

//hri
#include "inha_bt_pkg/hri/Listen.h"
#include "inha_bt_pkg/hri/Speak.h"
#include "inha_bt_pkg/hri/VlmQuery.h"
#include "inha_bt_pkg/hri/ExtractWord.h"
#include "inha_bt_pkg/hri/TextContainCondition.h"
#include "inha_bt_pkg/hri/ContextJsonMaker.h"

//perception
#include "inha_bt_pkg/perception/Waving.h"
#include "inha_bt_pkg/perception/Face.h"
#include "inha_bt_pkg/perception/WavingApproach.h"
#include "inha_bt_pkg/perception/ObjectPointcloud.h"
#include "inha_bt_pkg/perception/ObjectSwitch.h"
#include "inha_bt_pkg/perception/ActionFalse.h"
#include "inha_bt_pkg/perception/VisualAlign.h"
#include "inha_bt_pkg/perception/StopReplay.h"
#include "inha_bt_pkg/perception/StartReplay.h"

//manipulation
#include "inha_bt_pkg/manipulation/SetRobotPose.h"  
#include "inha_bt_pkg/manipulation/MoveitEnable.h"  
#include "inha_bt_pkg/manipulation/ExecuteSuccessServer.h"  
#include "inha_bt_pkg/manipulation/GraspgenEnable.h"  

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_node");

  auto black_board = BT::Blackboard::create();

  geometry_msgs::msg::PoseStamped origin;
  origin.header.frame_id = "map";
  origin.header.stamp = node->now();
  origin.pose.orientation.w = 1.0;

  black_board->set("origin_pose", origin);
  black_board->set("node", node);
  black_board->set("text", "None");
  black_board->set("prompt", "None");
  black_board->set("heard_text", "None");
  black_board->set("person_name", "None");
  black_board->set("favorite_drink", "None");
  black_board->set("vlm_output", "None");
  black_board->set("context_json", "None");


  BT::BehaviorTreeFactory factory;


  // Util Node
  ServiceTrigger::RegisterNodes(factory);

  // Navigation Node
  Exploration::RegisterNodes(factory);
  WaitGoalReached::RegisterNodes(factory);
  MakeNavGoal::RegisterNodes(factory);
  FollowHuman::RegisterNodes(factory);
  CmdPublish::RegisterNodes(factory);
  WaitBoolTopicBT::RegisterNodes(factory);
  GetCurrentPose::RegisterNodes(factory);
  ModifyPose::RegisterNodes(factory);
  GoToPose::RegisterNodes(factory);
  GoToPrecise::RegisterNodes(factory); 

  // Perception Node
  Waving::RegisterNodes(factory);
  FaceReg::RegisterNodes(factory);
  WavingApproach::RegisterNodes(factory);
  SetVisionDisable::RegisterNodes(factory);
  ObjectPointcloud::RegisterNodes(factory);
  ActionFalse::RegisterNodes(factory);
  VisualAlign::RegisterNodes(factory);
  StartReplay::RegisterNodes(factory);
  StopReplay::RegisterNodes(factory);

  // HRI Node
  Listen::RegisterNodes(factory);
  Speak::RegisterNodes(factory);
  VlmQuery::RegisterNodes(factory);
  ExtractWordBT::RegisterNodes(factory);
  TextContainCondition::RegisterNodes(factory);
  ContextJsonMaker::RegisterNodes(factory);

  // Manipulation Node
  SetRobotPose::RegisterNodes(factory);
  MoveitEnable::RegisterNodes(factory);
  ExecuteSuccessServer::RegisterNodes(factory);
  GraspgenEnable::RegisterNodes(factory);

  const std::string pkg_share = ament_index_cpp::get_package_share_directory("inha_bt_pkg");
  const std::string bt_root = pkg_share + "/bt_xmls/";

  // ✅ 기본값: missions/restaurant_main.xml
  std::string xml_rel = "missions/restaurant_main.xml";
  if (argc >= 2) {
    xml_rel = argv[1];  // 예: "missions/speak_test.xml"
  }

  const std::string xml_path = bt_root + xml_rel;
  RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_path.c_str());

  factory.registerBehaviorTreeFromFile(xml_path);

  // ✅ 트리 ID는 XML 안의 BehaviorTree ID와 같아야 함
  // restaurant_main.xml 안이 <BehaviorTree ID="restaurant_main"> 이면 아래 그대로
  auto main_tree = factory.createTree("main", black_board);

  RCLCPP_INFO(node->get_logger(), "Behavior Tree started!");

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);

  std::thread executor_thread([&](){ exec->spin(); });

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    main_tree.tickOnce();
    rate.sleep();
  }

  exec->cancel();
  rclcpp::shutdown();
  executor_thread.join();
}
