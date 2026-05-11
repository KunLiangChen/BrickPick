#pragma once
#include "behaviortree_cpp/action_node.h"
#include <vector>
#include <utility>

class ReadRegionCenters : public BT::SyncActionNode
{
public:
    ReadRegionCenters(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};