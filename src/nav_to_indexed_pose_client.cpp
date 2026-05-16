#include "brickpick/nav_to_indexed_pose_client.hpp"

NavToIndexedPoseClient::NavToIndexedPoseClient(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config), is_spinning_(false)
{
    // 1. 创建独立的 ROS 2 节点。
    // 建议节点名加上随机数或内存地址后缀，防止在行为树中实例化多个相同节点时发生命名冲突
    // std::string node_name = "nav_client_" + std::to_string(reinterpret_cast<uintptr_t>(this));
    node_ = std::make_shared<rclcpp::Node>(name);
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

    // 2. 将节点加入 Executor，并启动后台线程进行 Spin
    executor_.add_node(node_);
    is_spinning_ = true;
    spinner_thread_ = std::thread([this]() {
        while (rclcpp::ok() && is_spinning_) {
            executor_.spin_some(std::chrono::milliseconds(50));
        }
    });
}

NavToIndexedPoseClient::~NavToIndexedPoseClient()
{
    // 析构时：自管理生命周期结束，优雅关闭线程
    is_spinning_ = false;
    executor_.cancel(); // 中断可能在阻塞的 spin
    if (spinner_thread_.joinable()) {
        spinner_thread_.join();
    }
}

BT::NodeStatus NavToIndexedPoseClient::onStart()
{
    BT::Expected<int> index_opt = getInput<int>("current_index");
    BT::Expected<std::vector<std::pair<double, double>>> regions_opt = getInput<std::vector<std::pair<double, double>>>("regions");
    BT::Expected<std::string> frame_id_opt = getInput<std::string>("frame_id");

    if (!index_opt || !regions_opt || !frame_id_opt) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input ports.");
        return BT::NodeStatus::FAILURE;
    }

    int current_index = index_opt.value();
    auto regions = regions_opt.value();
    
    if (current_index < 0 || static_cast<size_t>(current_index) >= regions.size()) {
        RCLCPP_ERROR(node_->get_logger(), "Index out of bounds.");
        return BT::NodeStatus::FAILURE;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available!");
        return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = frame_id_opt.value();
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.pose.position.x = regions[current_index].first;
    goal_msg.pose.pose.position.y = regions[current_index].second;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(node_->get_logger(), "Sending goal to Nav2: index %d (%.2f, %.2f)", 
                current_index, goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    // 重置 promise 和 future
    result_promise_ = std::make_shared<std::promise<rclcpp_action::ResultCode>>();
    result_future_ = result_promise_->get_future();
    goal_handle_.reset(); // 清空上一轮的 handle

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    
    // 【修复点 1】：捕获 Goal Handle，这是 onHalted 能否成功取消任务的关键！
    send_goal_options.goal_response_callback = [this](const std::shared_ptr<GoalHandle>& handle) {
        if (!handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        } else {
            this->goal_handle_ = handle;
        }
    };

    // 接收结果回调
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result) {
        if (this->result_promise_) {
            this->result_promise_->set_value(result.code);
        }
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavToIndexedPoseClient::onRunning()
{
    // 如果 future 准备好了，说明 result_callback 被触发了
    if (result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        rclcpp_action::ResultCode code = result_future_.get();
        
        if (code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Nav2 succeeded!");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Nav2 failed or was canceled!");
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void NavToIndexedPoseClient::onHalted()
{
    // 如果行为树被强行打断，且目标已经被服务器接收（有了 handle）
    if (goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "Canceling Nav2 goal due to BT halt...");
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    
    // 丢弃当前的 promise，防止悬空引用
    result_promise_.reset();
}