# brickpick/launch/arm_preset.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_dir = get_package_share_directory('brickpick')
    arm_config = PathJoinSubstitution([pkg_dir, 'config', 'arm_presets.yaml'])
    vision_config = PathJoinSubstitution([pkg_dir, 'config', 'vision_params.yaml'])
    approach_config = PathJoinSubstitution([pkg_dir, 'config', 'approach_params.yaml'])
    arm_config      = PathJoinSubstitution([pkg_dir, 'config', 'arm_presets.yaml'])
    bt_xml_path     = PathJoinSubstitution([pkg_dir, 'config', 'brickpick_tree.xml'])
    # arm_node = Node(
    #     package='brickpick',
    #     executable='arm_preset_node.py',
    #     name='brickpick_arm_preset',
    #     output='screen',
    #     parameters=[arm_config]  
    # )

    vision_node = Node(
        package='brickpick',
        executable='vision_node.py',
        name='brickpick_vision',
        output='screen',
        parameters=[vision_config]
    )
    # 🔹 2. 搜寻节点 (受 BT 控制，服务/话题路径由 name 决定)
    find_node = Node(
        package='brickpick',
        executable='find_node.py',
        name='find_node',  # ⚠️ 必须与 C++ 中 /find_node/start 和 /find_node/status 一致
        output='screen',
        parameters=[{'rotate_speed': 0.6}]
    )

    # 🔹 3. 逼近节点 (受 BT 控制)
    approach_node = Node(
        package='brickpick',
        executable='approach_node.py',
        name='approach_node',  # ⚠️ 必须与 C++ 中 /approach_node/start 和 /approach_node/status 一致
        output='screen',
        parameters=[approach_config]
    )

    # 🔹 4. 机械臂节点 (受 BT 控制)
    arm_node = Node(
        package='brickpick',
        executable='arm_preset_node.py',
        name='arm_preset_node',  # ⚠️ 必须与 C++ 中 /arm_preset_node/start 和 /arm_preset_node/status 一致
        output='screen',
        parameters=[arm_config]
    )

    # 🔹 5. BT 执行器 (驱动单次流程：Find → Approach → Arm)
    bt_executor_node = Node(
        package='brickpick',
        executable='bt_executor',
        name='brickpick_bt_executor',
        output='screen',
        parameters=[
            {'bt_xml_path': bt_xml_path}  # XML 路径通过参数注入
        ]
    )

    return LaunchDescription([
        vision_node,
        find_node,
        approach_node,
        arm_node,
        bt_executor_node
    ])