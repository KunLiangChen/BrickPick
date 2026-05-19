#include "brickpick/has_next_center_point.hpp"

HasNextCenterPoint::HasNextCenterPoint(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList HasNextCenterPoint::providedPorts()
{
    return {
        BT::InputPort<int>("current_index"),
        BT::InputPort<std::vector<std::pair<double, double>>>("regions"),
    };
}

BT::NodeStatus HasNextCenterPoint::tick()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    auto logger = node->get_logger();

    BT::Expected<int> index_opt = getInput<int>("current_index");
    BT::Expected<std::vector<std::pair<double, double>>> regions_opt =
        getInput<std::vector<std::pair<double, double>>>("regions");

    if (!index_opt || !regions_opt) {
        RCLCPP_ERROR(logger,
                     "HasNextCenterPoint: Missing or invalid input port!");
        return BT::NodeStatus::FAILURE;
    }

    int current_index = index_opt.value();
    size_t total_regions = regions_opt.value().size();

    if (current_index >= 0
        && static_cast<size_t>(current_index) < total_regions) {
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(logger,
                "HasNextCenterPoint: Reached the end. Index: %d, Total: %zu",
                current_index, total_regions);
    return BT::NodeStatus::FAILURE;
}
