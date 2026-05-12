#include "brickpick/reading_region.hpp" // 确保名字对上
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>

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
    std::string pkg_share_dir;
    
    // 1. 获取包的绝对路径
    try {
        pkg_share_dir = ament_index_cpp::get_package_share_directory("brickpick");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"), "找不到 brickpick 包的共享目录: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }

    std::string file_path = pkg_share_dir + "/map/region_centers.txt";
    RCLCPP_INFO(rclcpp::get_logger("ReadRegionCenters"), "尝试打开文件: %s", file_path.c_str());

    // 2. 打开文件
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"), "无法打开文件: %s", file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // 3. 🌟 真正的文件解析逻辑 🌟
    std::string line;
    bool data_found = false;

    while (std::getline(file, line)) {
        // 跳过空行和注释行（以 # 开头）
        if (line.empty() || line[0] == '#') {
            continue;
        }

        double x, y;
        // 尝试读取两个 double 数字
        if (sscanf(line.c_str(), "%lf %lf", &x, &y) == 2) {
            regions.emplace_back(x, y);
            data_found = true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ReadRegionCenters"), "无法解析行: %s", line.c_str());
        }
    }

    // 4. 检查是否读到了数据
    if (!data_found) {
        RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"), "文件中没有有效坐标！");
        return BT::NodeStatus::FAILURE;
    }

    // 5. 写入黑板
    setOutput("regions", regions);
    setOutput("current_index", current_index);
    
    RCLCPP_INFO(rclcpp::get_logger("ReadRegionCenters"), "成功加载 %zu 个区域中心！", regions.size());
    
    return BT::NodeStatus::SUCCESS;
}