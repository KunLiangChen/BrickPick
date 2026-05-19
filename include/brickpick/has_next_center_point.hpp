#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <utility>

class HasNextCenterPoint : public BT::ConditionNode
{
public:
    HasNextCenterPoint(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};