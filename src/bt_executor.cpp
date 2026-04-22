#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
// #include <behaviortree_cpp/loggers/bt_zmq_publisher.h> // 可选：用于 BT 可视化
#include "brickpick/find_action.hpp"
#include "brickpick/approach_action.hpp"
#include "brickpick/arm_action.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto executor_node = rclcpp::Node::make_shared("brickpick_bt_executor");
    RCLCPP_INFO(executor_node->get_logger(), "BehaviorTree Executor 启动...");

    // 🔹 1. 加载 XML 路径（支持 launch 覆盖）
    executor_node->declare_parameter<std::string>("bt_xml_path", "config/brickpick_tree.xml");
    std::string xml_path = executor_node->get_parameter("bt_xml_path").as_string();

    // 🔹 2. 创建 Factory 并注册自定义 ActionNode
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT::FindAction>("FindObject");
    factory.registerNodeType<BT::ApproachAction>("ApproachObject");
    factory.registerNodeType<BT::ArmAction>("ExecuteArmSequence");

    // 🔹 3. 从 XML 构建树
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_path);
        RCLCPP_INFO(executor_node->get_logger(), "成功加载 BT XML: %s", xml_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(executor_node->get_logger(), "加载 BT XML 失败: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }




    // 添加一个变量来记录树的状态
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    auto rate = std::make_shared<rclcpp::Rate>(50);

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        // 执行一次 tick，返回当前根节点的状态
        status = tree.tickOnce();
        
        // 处理 ROS 回调
        rclcpp::spin_some(executor_node);
        
        // 控制循环频率
        rate->sleep();
    }

    // 🔹 5. 输出最终状态
    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(executor_node->get_logger(), "任务成功完成！");
    } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(executor_node->get_logger(), "任务执行失败或被中止。");
    } else {
        RCLCPP_INFO(executor_node->get_logger(), "节点退出。");
    }

    rclcpp::shutdown();
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}