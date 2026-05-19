from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('brickpick')
    arm_config = PathJoinSubstitution([pkg_dir, 'config', 'arm_presets.yaml'])
    vision_config = PathJoinSubstitution(
        [pkg_dir, 'config', 'vision_params.yaml'])
    approach_config = PathJoinSubstitution(
        [pkg_dir, 'config', 'approach_params.yaml'])
    bt_xml_path = PathJoinSubstitution(
        [pkg_dir, 'config', 'brickpick_tree.xml'])

    vision_node = Node(
        package='brickpick',
        executable='vision_node.py',
        name='brickpick_vision',
        output='screen',
        parameters=[vision_config],
    )

    find_node = Node(
        package='brickpick',
        executable='find_node.py',
        name='find_node',
        output='screen',
        parameters=[{'rotate_speed': 0.6}],
    )

    approach_node = Node(
        package='brickpick',
        executable='approach_node.py',
        name='approach_node',
        output='screen',
        parameters=[approach_config],
    )

    arm_node = Node(
        package='brickpick',
        executable='arm_preset_node.py',
        name='arm_preset_node',
        output='screen',
        parameters=[arm_config],
    )

    bt_executor_node = Node(
        package='brickpick',
        executable='bt_executor',
        name='brickpick_bt_executor',
        output='screen',
        parameters=[{'bt_xml_path': bt_xml_path}],
    )

    return LaunchDescription([
        vision_node,
        find_node,
        approach_node,
        arm_node,
        bt_executor_node,
    ])
