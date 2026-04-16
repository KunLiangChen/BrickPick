# brickpick/launch/arm_preset.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_dir = get_package_share_directory('brickpick')
    config_file = PathJoinSubstitution([pkg_dir, 'config', 'arm_presets.yaml'])
    
    arm_node = Node(
        package='brickpick',
        executable='arm_preset_node.py',
        name='brickpick_arm_preset',
        output='screen',
        parameters=[config_file]  
    )
    # Currently we only have arm preset
    return LaunchDescription([arm_node])