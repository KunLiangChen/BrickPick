#include <behaviortree_cpp/bt_factory.h>
#include "brickpick/find_action.hpp"
#include "brickpick/approach_action.hpp"
#include "brickpick/arm_action.hpp"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<BT::FindAction>("FindObject");
    factory.registerNodeType<BT::ApproachAction>("ApproachObject");
    factory.registerNodeType<BT::ArmAction>("ExecuteArmSequence");
}
