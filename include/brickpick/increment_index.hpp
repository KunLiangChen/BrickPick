#pragma once

#include "behaviortree_cpp/action_node.h"

class IncrementIndex : public BT::SyncActionNode
{
public:
    IncrementIndex(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};