#include "inha_bt_pkg/hri/ContextJsonMaker.h"

#include <nlohmann/json.hpp>

namespace ContextJsonMaker
{

ContextJsonMaker::ContextJsonMaker(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{
  // optional: node handle for logging (if provided in blackboard)
  if (config.blackboard) {
    try {
      node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    } catch (...) {
      // node_ is optional
    }
  }
}

BT::PortsList ContextJsonMaker::providedPorts()
{
  return {
    BT::InputPort<std::string>("saved_dir", "", "directory path where face images are saved"),
    BT::InputPort<std::string>("person_name", "", "person name"),
    BT::InputPort<std::string>("favorite_drink", "", "favorite drink (optional)"),
    BT::InputPort<std::string>("extra_json", "", "extra json string merged into output (optional)"),
    BT::OutputPort<std::string>("context_json", "json string for VLM mode=3")
  };
}

BT::NodeStatus ContextJsonMaker::tick()
{
  const std::string saved_dir      = getInput<std::string>("saved_dir").value_or("");
  const std::string person_name    = getInput<std::string>("person_name").value_or("");
  const std::string favorite_drink = getInput<std::string>("favorite_drink").value_or("");
  const std::string extra_json     = getInput<std::string>("extra_json").value_or("");

  nlohmann::json j;
  if (!saved_dir.empty())      j["saved_dir"] = saved_dir;
  if (!person_name.empty())    j["person_name"] = person_name;
  if (!favorite_drink.empty()) j["favorite_drink"] = favorite_drink;

  // optional merge (string -> json)
  if (!extra_json.empty()) {
    try {
      auto extra = nlohmann::json::parse(extra_json);
      if (extra.is_object()) {
        for (auto it = extra.begin(); it != extra.end(); ++it) {
          j[it.key()] = it.value();
        }
      }
    } catch (const std::exception& e) {
      if (node_) {
        RCLCPP_WARN(node_->get_logger(), "[ContextJsonMaker] extra_json parse failed: %s", e.what());
      }
      // ignore extra_json parse errors (do not fail the BT)
    }
  }

  const std::string out = j.dump();  // compact json
  setOutput("context_json", out);

  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "[ContextJsonMaker] context_json len=%zu", out.size());
  }

  return BT::NodeStatus::SUCCESS;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ContextJsonMaker>("ContextJsonMaker");
}


}  // namespace ContextJsonMaker
