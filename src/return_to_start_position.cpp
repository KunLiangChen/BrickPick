#include "brickpick/return_to_start_position.hpp"

ReturnToStartPosition::ReturnToStartPosition(
    const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config), is_spinning_(false)
{
    node_ = std::make_shared<rclcpp::Node>(name);
    action_client_ = rclcpp_action::create_client<
        nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 10);

    executor_.add_node(node_);
    is_spinning_ = true;
    spinner_thread_ = std::thread([this]() {
        while (rclcpp::ok() && is_spinning_) {
            executor_.spin_some(std::chrono::milliseconds(50));
        }
    });
}

ReturnToStartPosition::~ReturnToStartPosition()
{
    is_spinning_ = false;
    executor_.cancel();
    if (spinner_thread_.joinable()) {
        spinner_thread_.join();
    }
}

BT::NodeStatus ReturnToStartPosition::onStart()
{
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available!");
        return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();

    goal_msg.pose.pose.position.x = -0.14;
    goal_msg.pose.pose.position.y = -0.15;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.198;
    goal_msg.pose.pose.orientation.w = 0.980;

    RCLCPP_INFO(node_->get_logger(),
                "Mission Complete! Returning to start position (%.2f, %.2f)",
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    result_promise_ =
        std::make_shared<std::promise<rclcpp_action::ResultCode>>();
    result_future_ = result_promise_->get_future();
    goal_handle_.reset();

    auto send_goal_options =
        rclcpp_action::Client<
            nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const std::shared_ptr<GoalHandle>& handle) {
            if (!handle) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Goal was rejected by server");
            } else {
                this->goal_handle_ = handle;
            }
        };

    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult &result) {
            if (this->result_promise_) {
                this->result_promise_->set_value(result.code);
            }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToStartPosition::onRunning()
{
    if (result_future_.wait_for(std::chrono::milliseconds(0))
        == std::future_status::ready) {
        rclcpp_action::ResultCode code = result_future_.get();
        if (code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(),
                        "Successfully returned to start position!");

            // Send duplicate stop commands to ensure the chassis receives
            // the brake signal before the program exits.
            geometry_msgs::msg::Twist stop_msg;
            cmd_vel_pub_->publish(stop_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            cmd_vel_pub_->publish(stop_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to return to start position!");
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void ReturnToStartPosition::onHalted()
{
    if (goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "Canceling return goal...");
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    result_promise_.reset();
}
