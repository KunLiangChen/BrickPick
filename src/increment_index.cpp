// increment_index.cpp
#include "increment_index.hpp"

IncrementIndex::IncrementIndex(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList IncrementIndex::providedPorts()
{
    return {
        // 🌟 关键点：BidirectionalPort 表示这个端口既能读也能写
        BT::BidirectionalPort<int>("current_index")
    };
}

BT::NodeStatus IncrementIndex::tick()
{
    // 1. 从端口（也就是黑板）读取当前的值
    BT::Expected<int> index_opt = getInput<int>("current_index");

    // 2. 错误检查：如果黑板里没有这个值，说明初始化有问题
    if (!index_opt)
    {
        return BT::NodeStatus::FAILURE;
    }

    // 3. 核心逻辑：加 1
    int new_index = index_opt.value() + 1;

    // 4. 将新值写回端口（写回黑板）
    // 因为声明的是 BidirectionalPort，所以可以用 setOutput 覆盖原值
    setOutput("current_index", new_index);

    return BT::NodeStatus::SUCCESS;
}