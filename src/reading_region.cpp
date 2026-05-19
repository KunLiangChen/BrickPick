#include "brickpick/reading_region.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>

ReadRegionCenters::ReadRegionCenters(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList ReadRegionCenters::providedPorts()
{
    return {
        BT::OutputPort<std::vector<std::pair<double, double>>>("regions"),
        BT::OutputPort<int>("current_index"),
    };
}

BT::NodeStatus ReadRegionCenters::tick()
{
    std::vector<std::pair<double, double>> regions;
    int current_index = 0;
    std::string pkg_share_dir;

    try {
        pkg_share_dir = ament_index_cpp::get_package_share_directory("brickpick");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"),
                     "Cannot locate brickpick package share directory: %s",
                     e.what());
        return BT::NodeStatus::FAILURE;
    }

    std::string file_path = pkg_share_dir + "/map/region_centers.txt";
    RCLCPP_INFO(rclcpp::get_logger("ReadRegionCenters"),
                "Attempting to open file: %s", file_path.c_str());

    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"),
                     "Cannot open file: %s", file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }

    std::string line;
    bool data_found = false;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        double x, y;
        if (sscanf(line.c_str(), "%lf %lf", &x, &y) == 2) {
            regions.emplace_back(x, y);
            data_found = true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ReadRegionCenters"),
                        "Failed to parse line: %s", line.c_str());
        }
    }

    if (!data_found) {
        RCLCPP_ERROR(rclcpp::get_logger("ReadRegionCenters"),
                     "No valid coordinates found in file.");
        return BT::NodeStatus::FAILURE;
    }

    setOutput("regions", regions);
    setOutput("current_index", current_index);

    RCLCPP_INFO(rclcpp::get_logger("ReadRegionCenters"),
                "Successfully loaded %zu region centers.", regions.size());

    return BT::NodeStatus::SUCCESS;
}
