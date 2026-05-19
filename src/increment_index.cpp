#include "brickpick/increment_index.hpp"

IncrementIndex::IncrementIndex(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList IncrementIndex::providedPorts()
{
    return {
        BT::BidirectionalPort<int>("current_index"),
    };
}

BT::NodeStatus IncrementIndex::tick()
{
    BT::Expected<int> index_opt = getInput<int>("current_index");

    if (!index_opt) {
        return BT::NodeStatus::FAILURE;
    }

    int new_index = index_opt.value() + 1;

    setOutput("current_index", new_index);

    return BT::NodeStatus::SUCCESS;
}
