// include necessary headers
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include <fstream>
#include <string>
#include <vector>
#include <utility> // for std::pair

class ReadRegionCenters : public BT::SyncActionNode
{
public:
    ReadRegionCenters(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}

    // 这个节点不需要任何端口，直接操作黑板
    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        // 1. 获取黑板指针（通过config().blackboard）
        auto blackboard = config().blackboard;

        // 2. 打开并读取文件
        std::ifstream file("./map/region_centers.txt");
        if (!file.is_open()) {
            BT_ROS_ERROR("Could not open file: ./map/region_centers.txt");
            return BT::NodeStatus::FAILURE;
        }

        std::vector<std::pair<double, double>> regions;
        std::string line;

        // 跳过注释行
        std::getline(file, line);

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            double x, y;
            if (sscanf(line.c_str(), "%lf %lf", &x, &y) == 2) {
                regions.emplace_back(x, y);
            }
        }

        // 3. 将数据存入黑板
        blackboard->set("regions", regions);
        blackboard->set("current_index", 0); // 初始化索引为0

        BT_ROS_INFO("Successfully loaded %zu region centers into blackboard.", regions.size());
        return BT::NodeStatus::SUCCESS;
    }
};