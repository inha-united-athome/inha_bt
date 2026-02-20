#include "inha_bt_pkg/utils/ServiceTrigger.h"

#include <chrono>

namespace ServiceTrigger
{

ServiceTrigger::ServiceTrigger(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw BT::RuntimeError("ServiceTrigger: missing 'node' in blackboard");
  }
}

rclcpp::Client<ServiceTrigger::SrvT>::SharedPtr
ServiceTrigger::getClientLocked(const std::string& srv_name)
{
  auto it = clients_.find(srv_name);
  if (it != clients_.end()) {
    return it->second;
  }

  auto client = node_->create_client<SrvT>(srv_name);
  clients_[srv_name] = client;
  return client;
}

BT::NodeStatus ServiceTrigger::tick()
{
  const std::string srv_name =
    getInput<std::string>("service_name").value_or("/manipulation/graspgen/enable");

  const bool enable = getInput<bool>("enable").value_or(true);

  if (srv_name.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[ServiceTrigger] service_name is empty");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::Client<SrvT>::SharedPtr client;
  {
    std::lock_guard<std::mutex> lk(mu_);
    client = getClientLocked(srv_name);
  }

  if (!client->wait_for_service(std::chrono::milliseconds(300))) {
    RCLCPP_WARN(node_->get_logger(), "[ServiceTrigger] service not available: %s", srv_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<SrvT::Request>();
  req->enable = enable;

  auto future = client->async_send_request(req);

  if (future.wait_for(std::chrono::milliseconds(800)) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(), "[ServiceTrigger] timeout calling: %s", srv_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_WARN(node_->get_logger(), "[ServiceTrigger] failed: %s (%s, enable=%s)",
                resp->message.c_str(), srv_name.c_str(), enable ? "true" : "false");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[ServiceTrigger] success: %s (%s, enable=%s)",
              resp->message.c_str(), srv_name.c_str(), enable ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
}


void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ServiceTrigger>("ServiceTrigger");
}

} // namespace ServiceTrigger
