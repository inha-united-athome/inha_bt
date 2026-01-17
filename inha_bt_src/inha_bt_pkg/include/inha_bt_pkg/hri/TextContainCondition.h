#include <behaviortree_cpp/condition_node.h>
#include <algorithm>
#include <cctype>
#include <string>

class TextContainCondition : public BT::ConditionNode
{
public:
  TextContainCondition(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("text", "", "Input text"),
      BT::InputPort<std::string>("keyword", "", "Single keyword to find"),
      BT::InputPort<bool>("case_insensitive", true, "Case-insensitive")
    };
  }

  BT::NodeStatus tick() override
  {
    auto t = getInput<std::string>("text");
    auto k = getInput<std::string>("keyword");
    const bool ci = getInput<bool>("case_insensitive").value_or(true);

    if (!t || !k || k->empty()) return BT::NodeStatus::FAILURE;

    std::string text = *t;
    std::string key  = *k;

    if (ci) {
      toLowerInPlace(text);
      toLowerInPlace(key);
    }

    return (text.find(key) != std::string::npos)
             ? BT::NodeStatus::SUCCESS   // True
             : BT::NodeStatus::FAILURE;  // False
  }

  static void RegisterNodes(BT::BehaviorTreeFactory& factory)
  {
    factory.registerNodeType<TextContainCondition>("TextContainCondition");
  }

private:
  static void toLowerInPlace(std::string& s)
  {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); });
  }
};
