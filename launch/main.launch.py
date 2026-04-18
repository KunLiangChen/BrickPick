# brickpick/launch/arm_preset.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_dir = get_package_share_directory('brickpick')
    arm_config = PathJoinSubstitution([pkg_dir, 'config', 'arm_presets.yaml'])
    vision_config = PathJoinSubstitution([pkg_dir, 'config', 'vision_params.yaml'])
    
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
    
    return LaunchDescription([
        # arm_node,
        vision_node
    ])