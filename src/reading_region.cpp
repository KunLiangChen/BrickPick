#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <optional>

class ReadRegionCenters : public BT::SyncActionNode
{
public:
    ReadRegionCenters(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}

    // 关键改进：声明输出端口
    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::vector<std::pair<double, double>>>("regions"),
            BT::OutputPort<int>("current_index")
        };
    }

    BT::NodeStatus tick() override
    {
        // 1. 准备输出数据
        std::vector<std::pair<double, double>> regions;
        int current_index = 0;

        // 2. 打开并读取文件
        std::ifstream file("./map/region_centers.txt");
        if (!file.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"), "无法打开文件: ./map/region_centers.txt");
            return BT::NodeStatus::FAILURE;
        }

        std::string line;
        bool data_found = false;

        // 跳过注释行
        std::getline(file, line);

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue; // 跳过空行和注释

            double x, y;
            if (sscanf(line.c_str(), "%lf %lf", &x, &y) == 2) {
                regions.emplace_back(x, y);
                data_found = true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("ReadRegionCenters"), "无法解析行: %s", line.c_str());
            }
        }

        // 3. 检查是否读取到有效数据
        if (!data_found) {
            RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"), "文件中未找到有效的区域坐标数据。");
            return BT::NodeStatus::FAILURE;
        }

        // 4. 将数据通过输出端口设置到黑板
        // setOutput 方法会自动处理端口与黑板键的映射【turn0search3】
        setOutput("regions", regions);
        setOutput("current_index", current_index);

        RCLCPP_INFO(rclcpp::get_logger("ReadRegionCenters"), "成功加载 %zu 个区域中心到黑板。", regions.size());
        return BT::NodeStatus::SUCCESS;
    }
};