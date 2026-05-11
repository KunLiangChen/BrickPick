// has_next_center_point.hpp
#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <vector>
#include <utility> // for std::pair

class HasNextCenterPoint : public BT::ConditionNode
{
public:
    // 构造函数声明
    HasNextCenterPoint(const std::string& name, const BT::NodeConfiguration& config);

    // 声明该节点需要的输入端口（不包含具体逻辑）
    static BT::PortsList providedPorts();

    // 核心执行逻辑的声明
    BT::NodeStatus tick() override;
};