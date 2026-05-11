// increment_index.hpp
#pragma once

#include "behaviortree_cpp/action_node.h"

class IncrementIndex : public BT::SyncActionNode
{
public:
    IncrementIndex(const std::string& name, const BT::NodeConfiguration& config);

    // 声明双向端口
    static BT::PortsList providedPorts();

    // 执行逻辑
    BT::NodeStatus tick() override;
};