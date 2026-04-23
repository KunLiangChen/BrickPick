// src/bt_nodes_plugin.cpp
#include <behaviortree_cpp/bt_factory.h>
#include "brickpick/find_action.hpp"
#include "brickpick/approach_action.hpp"
#include "brickpick/arm_action.hpp"

// 🔑 Groot2 动态加载的入口宏
BT_REGISTER_NODES(factory)
{
    // 注册名称必须与 brickpick_tree.xml 中的 ID 一致！
    factory.registerNodeType<BT::FindAction>("FindObject");
    factory.registerNodeType<BT::ApproachAction>("ApproachObject");
    factory.registerNodeType<BT::ArmAction>("ExecuteArmSequence");
}