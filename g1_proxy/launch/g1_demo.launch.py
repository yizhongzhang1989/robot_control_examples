# Heecheol Kim | heecheolkim@microsoft.com | MSRA Tokyo | 2025-12-23
# Includes g1_dual_proxy.launch.py and additionally launches g1_node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('g1_proxy')

    # Include the existing dual proxy launch file (ros2_control nodes)
    dual_proxy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'g1_dual_proxy.launch.py'])
        )
    )

    # G1 Node (Interface with Real Robot via DDS)
    g1_node = Node(
        package='g1_proxy',
        executable='g1_node',
        name='g1_node',
        output='screen'
    )

    return LaunchDescription([
        dual_proxy_launch,
        g1_node
    ])
