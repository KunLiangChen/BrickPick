#include "brickpick/reading_region.hpp" // 确保名字对上
#include "rclcpp/rclcpp.hpp"
#include <fstream>

// 构造函数实现
ReadRegionCenters::ReadRegionCenters(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{}

// 端口列表实现
BT::PortsList ReadRegionCenters::providedPorts()
{
    return {
        BT::OutputPort<std::vector<std::pair<double, double>>>("regions"),
        BT::OutputPort<int>("current_index")
    };
}

// Tick逻辑实现
BT::NodeStatus ReadRegionCenters::tick()
{
    std::vector<std::pair<double, double>> regions;
    int current_index = 0;

    std::ifstream file("./map/region_centers.txt");
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"), "无法打开文件");
        return BT::NodeStatus::FAILURE;
    }

    // ... 解析逻辑保持不变，但要确保在 ReadRegionCenters::tick() 作用域内 ...
    
    setOutput("regions", regions);
    setOutput("current_index", current_index);
    return BT::NodeStatus::SUCCESS;
}