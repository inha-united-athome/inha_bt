#include "inha_bt_pkg/perception/Face.h"

#include <filesystem>
#include <iostream>
#include <algorithm>
#include <inha_interfaces/action/capture_face_crop.hpp>

// -------------------------
// 전역 맵 정의 (.cpp에서 단 한 번)
// -------------------------
std::unordered_map<std::string, PersonInfo> g_people;
std::mutex g_map_mx;

// ========================= WaitPersonDetected =========================
WaitPersonDetected::WaitPersonDetected(const std::string& name, const BT::NodeConfig& cfg)
: SimpleActionBT<inha_interfaces::action::WaitPerson>(name, cfg)
{
  client_ = rclcpp_action::create_client<inha_interfaces::action::WaitPerson>(
      node_, "/wait_person");
}

BT::NodeStatus WaitPersonDetected::onStart()
{
  const int timeout_ms      = getInputOr<int>("timeout_ms", 8000);

  if (!client_ || !client_->wait_for_action_server(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(node_->get_logger(), "[WaitPersonDetected] wait_person server not ready");
    return BT::NodeStatus::FAILURE;
  }

  WaitPerson::Goal goal;
  goal.timeout_ms = timeout_ms;

  goal_handle_future_ = client_->async_send_goal(goal);
  started_ = true;
  goal_handle_.reset();
  result_future_ = {};

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitPersonDetected::onRunning()
{
  if (!started_) return BT::NodeStatus::FAILURE;

  // 1) goal handle 확보
  if (!goal_handle_) {
    if (goal_handle_future_.valid() &&
        goal_handle_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      goal_handle_ = goal_handle_future_.get();
      if (!goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "[WaitPersonDetected] goal rejected");
        return BT::NodeStatus::FAILURE;
      }
      result_future_ = client_->async_get_result(goal_handle_);
    }
    return BT::NodeStatus::RUNNING;
  }

  // 2) result 확보
  if (result_future_.valid() &&
      result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
  {
    auto wrapped = result_future_.get();
    auto res = wrapped.result;

    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && res) {
      return (res->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitPersonDetected::onHalted()
{
  if (client_ && goal_handle_) {
    (void)client_->async_cancel_goal(goal_handle_);
  }
  started_ = false;
  goal_handle_.reset();
}

// ========================= Mapstore =========================
Mapstore::Mapstore(const std::string& name, const BT::NodeConfig& cfg)
: BT::SyncActionNode(name, cfg)
{
  try { node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node"); }
  catch(...) { node_.reset(); }
}

BT::NodeStatus Mapstore::tick()
{
  auto n     = getInput<std::string>("person");
  auto drink = getInput<std::string>("favorite_drink");
  auto id    = getInput<int>("person_id");

  if (!n || n->empty()) {
    if (node_) RCLCPP_WARN(node_->get_logger(), "[Mapstore] missing or empty person name");
    else std::cerr << "[Mapstore] missing or empty person name\n";
    return BT::NodeStatus::FAILURE;
  }

  bool person_id_exists = false;
  {
    std::lock_guard<std::mutex> lk(g_map_mx);
    person_id_exists = (g_people.find(*n) != g_people.end());
  }

  if (id) {
    if (!person_id_exists) {
      std::lock_guard<std::mutex> lk(g_map_mx);
      g_people[*n] = PersonInfo();
      g_people[*n].person_id = *id;
    }
  } else {
    if (person_id_exists && node_) {
      RCLCPP_INFO(node_->get_logger(), "[Mapstore] Existing person found: %s", n->c_str());
    }
  }

  // desktop root
  std::string desktop_root = "/home/thor/face_ws/src/face_data";
  if (node_) {
    if (!node_->has_parameter("face_db_desktop_root")) {
      node_->declare_parameter<std::string>("face_db_desktop_root", desktop_root);
    }
    desktop_root = node_->get_parameter("face_db_desktop_root").as_string();
  }

  const std::string person_name = *n;
  const std::string desktop_dir = desktop_root + "/" + person_name;

  {
    std::lock_guard<std::mutex> lk(g_map_mx);
    auto& info = g_people[person_name];
    info.name = person_name;
    info.saved_dir = desktop_dir;
    if (drink && !drink->empty()) info.favorite_drink = *drink;
  }

  return BT::NodeStatus::SUCCESS;
}

// ========================= FaceSaver =========================
FaceSaver::FaceSaver(const std::string& name, const BT::NodeConfig& cfg)
: SimpleActionBT<inha_interfaces::action::CaptureFaceCrop>(name, cfg)
{
  client_ = rclcpp_action::create_client<inha_interfaces::action::CaptureFaceCrop>(
      node_, "capture_face_crop");
}

BT::NodeStatus FaceSaver::onStart()
{
  auto p = getInput<std::string>("person_name");
  if (!p || trim(*p).empty()) {
    RCLCPP_WARN(node_->get_logger(), "[FaceSaver] person_name is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_ || !client_->wait_for_action_server(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(node_->get_logger(), "[FaceSaver] action server not ready");
    return BT::NodeStatus::FAILURE;
  }

  CaptureFaceCrop::Goal goal;
  goal.person_name = trim(*p);
  goal.num_images  = std::max(1, getInputOr<int>("num_images", 1));
  goal.timeout_ms  = std::max(1, getInputOr<int>("timeout_ms", 3000));

  // ✅ goal 전송하고, goal handle future만 저장
  goal_handle_future_ = client_->async_send_goal(goal);
  goal_handle_.reset();
  result_future_ = {};   // 비움
  started_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FaceSaver::onRunning()
{
  if (!started_) return BT::NodeStatus::FAILURE;

  // 1) goal_handle 받기
  if (!goal_handle_) {
    if (goal_handle_future_.valid() &&
        goal_handle_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      goal_handle_ = goal_handle_future_.get();
      if (!goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "[FaceSaver] goal rejected");
        started_ = false;
        return BT::NodeStatus::FAILURE;
      }
      result_future_ = client_->async_get_result(goal_handle_);
    }
    return BT::NodeStatus::RUNNING;
  }

  // 2) result 받기
  if (result_future_.valid() &&
      result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
  {
    auto wrapped = result_future_.get();
    started_ = false;

    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result) {
      return wrapped.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void FaceSaver::onHalted()
{
  if (client_ && goal_handle_) {
    (void)client_->async_cancel_goal(goal_handle_);
  }
  started_ = false;
  goal_handle_.reset();
  result_future_ = {};
}


// ========================= Factory 등록 =========================
namespace FaceReg {

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<WaitPersonDetected>("WaitPersonDetected");
  factory.registerNodeType<Mapstore>("Mapstore");
  factory.registerNodeType<FaceSaver>("FaceSaver");
}

} // namespace FaceReg
