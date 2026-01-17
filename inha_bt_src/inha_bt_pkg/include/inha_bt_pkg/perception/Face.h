#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <unordered_map>
#include <mutex>
#include <string>
#include <cctype>


#include <inha_interfaces/action/wait_person.hpp> 
#include <inha_interfaces/action/capture_face_crop.hpp>
// ------------------------------------------------------------
// util
// ------------------------------------------------------------
static inline std::string trim(std::string s)
{
  auto sp = [](unsigned char c) { return std::isspace(c); };
  while (!s.empty() && sp(s.front())) s.erase(s.begin());
  while (!s.empty() && sp(s.back()))  s.pop_back();
  return s;
}

// ============================================================
// 공용 베이스: Action future polling 최소 구현용
// ============================================================
template<typename ActionT>
class SimpleActionBT : public BT::StatefulActionNode
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using ClientT    = rclcpp_action::Client<ActionT>;

  SimpleActionBT(const std::string& name, const BT::NodeConfig& cfg)
  : BT::StatefulActionNode(name, cfg)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

protected:
  template<typename T>
  T getInputOr(const std::string& key, const T& def)
  {
    auto v = getInput<T>(key);
    return v ? *v : def;
  }

  void resetActionState()
  {
    started_ = false;
    goal_handle_.reset();
    goal_handle_future_ = {};
    result_future_ = {};
  }

  rclcpp::Node::SharedPtr node_;
  typename ClientT::SharedPtr client_;

  std::shared_future<typename GoalHandle::SharedPtr> goal_handle_future_;
  typename GoalHandle::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandle::WrappedResult> result_future_;

  bool started_{false};
};

// ============================================================
// 1) WaitPersonDetected  (사람 감지 액션)
// ============================================================
class WaitPersonDetected : public SimpleActionBT<inha_interfaces::action::WaitPerson>
{
public:
  using WaitPerson = inha_interfaces::action::WaitPerson;

  static BT::PortsList providedPorts()
  {
    return {
      // ✅ 얘는 FaceSaver 포트가 아니라 기다리기용 포트여야 함
      BT::InputPort<int>("timeout_ms", 8000, "timeout in ms"),
    };
  }

  WaitPersonDetected(const std::string& name, const BT::NodeConfig& cfg);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

// ============================================================
// 2) PersonInfo & global map
// ============================================================
struct PersonInfo
{
  std::string name;
  int         person_id = -1;
  std::string saved_dir;
  std::string favorite_drink;
};

extern std::unordered_map<std::string, PersonInfo> g_people;
extern std::mutex g_map_mx;

// ============================================================
// 3) Mapstore
// ============================================================
class Mapstore : public BT::SyncActionNode
{
public:
  Mapstore(const std::string& name, const BT::NodeConfig& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("person"),
      BT::InputPort<int>("person_id", 0, "ID of the person"),               // ✅ int default는 3인자
      BT::InputPort<std::string>("favorite_drink", "", "favorite drink"),   // ok
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// ============================================================
// 4) FaceSaver  (캡쳐 액션)
// ============================================================
class FaceSaver : public SimpleActionBT<inha_interfaces::action::CaptureFaceCrop>
{
public:
  using CaptureFaceCrop = inha_interfaces::action::CaptureFaceCrop;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("person_name", "", "person name for folder"),
      BT::InputPort<int>("num_images", 1, "number of images to save"),  // ✅ 3인자
      BT::InputPort<int>("timeout_ms", 3000, "timeout in ms"),          // ✅ 3인자
    };
  }

  FaceSaver(const std::string& name, const BT::NodeConfig& cfg);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

// ============================================================
// 5) Factory register
// ============================================================
namespace FaceReg {
void RegisterNodes(BT::BehaviorTreeFactory& factory);
}
