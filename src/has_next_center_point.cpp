// has_next_center_point.cpp
#include "brickpick/has_next_center_point.hpp"

// ---------------------- 构造函数实现 ----------------------
HasNextCenterPoint::HasNextCenterPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{}

// ---------------------- 端口声明实现 ----------------------
BT::PortsList HasNextCenterPoint::providedPorts()
{
    return {
        BT::InputPort<int>("current_index"),
        BT::InputPort<std::vector<std::pair<double, double>>>("regions")
    };
}

// ---------------------- 核心逻辑实现 ----------------------
BT::NodeStatus HasNextCenterPoint::tick()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    auto logger = node->get_logger();
    // 1. 从端口安全地获取数据
    BT::Expected<int> index_opt = getInput<int>("current_index");
    BT::Expected<std::vector<std::pair<double, double>>> regions_opt = 
        getInput<std::vector<std::pair<double, double>>>("regions");

    // 2. 检查数据是否有效
    if (!index_opt || !regions_opt)
    {
        RCLCPP_ERROR(logger,"HasNextCenterPoint: Missing or invalid input port!");
        return BT::NodeStatus::FAILURE;
    }

    int current_index = index_opt.value();
    size_t total_regions = regions_opt.value().size();

    // 3. 判断逻辑：索引是否在合法范围内 [0, size-1]
    if (current_index >= 0 && static_cast<size_t>(current_index) < total_regions)
    {
        // 还没遍历完，返回 SUCCESS
        return BT::NodeStatus::SUCCESS;
    }

    // 已经越界，说明所有点都遍历完了
    RCLCPP_INFO(logger, "HasNextCenterPoint: Reached the end. Index: %d, Total: %zu", 
            current_index, total_regions);
    return BT::NodeStatus::FAILURE;
}