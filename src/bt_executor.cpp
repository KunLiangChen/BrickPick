#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "brickpick/find_action.hpp"
#include "brickpick/approach_action.hpp"
#include "brickpick/arm_action.hpp"
#include "brickpick/nav_to_indexed_pose_client.hpp"
#include "brickpick/has_next_center_point.hpp"
#include "brickpick/reading_region.hpp"
#include "brickpick/increment_index.hpp"
#include "brickpick/return_to_start_position.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto executor_node = rclcpp::Node::make_shared("brickpick_bt_executor");
    RCLCPP_INFO(executor_node->get_logger(),
                "BehaviorTree Executor starting...");

    executor_node->declare_parameter<std::string>(
        "bt_xml_path", "config/brickpick_tree.xml");
    std::string xml_path = executor_node->get_parameter(
        "bt_xml_path").as_string();

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT::FindAction>("FindObject");
    factory.registerNodeType<BT::ApproachAction>("ApproachObject");
    factory.registerNodeType<BT::ArmAction>("ExecuteArmSequence");
    factory.registerNodeType<NavToIndexedPoseClient>(
        "NavToIndexedPoseClient");
    factory.registerNodeType<HasNextCenterPoint>("HasNextCenterPoint");
    factory.registerNodeType<ReadRegionCenters>("ReadRegionCenters");
    factory.registerNodeType<IncrementIndex>("IncrementIndex");
    factory.registerNodeType<ReturnToStartPosition>(
        "ReturnToStartPosition");

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_path);
        tree.rootBlackboard()->set<rclcpp::Node::SharedPtr>(
            "node", executor_node);
        RCLCPP_INFO(executor_node->get_logger(),
                    "Successfully loaded BT XML: %s", xml_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(executor_node->get_logger(),
                     "Failed to load BT XML: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    auto rate = std::make_shared<rclcpp::Rate>(50);

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        rclcpp::spin_some(executor_node);
        rate->sleep();
    }

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(executor_node->get_logger(),
                    "Mission completed successfully.");
    } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(executor_node->get_logger(),
                    "Mission failed or was aborted.");
    } else {
        RCLCPP_INFO(executor_node->get_logger(), "Node exiting.");
    }

    rclcpp::shutdown();
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
