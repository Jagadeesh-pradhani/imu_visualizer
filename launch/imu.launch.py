import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    pub_frame = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'imu_link']
    )
    ld.add_action(pub_frame)
    
    # Robot State Publisher node
    imu_node = Node(
        package='imu_visualizer',
        executable='imu_viz',
        name='imu_viz',
        output='screen'
    )
    ld.add_action(imu_node)

    # Rviz2 node
    rviz_config_path = os.path.join(get_package_share_directory('imu_visualizer'), 'rviz', 'test2.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='xsens_rviz2',
        output='screen',
        arguments=[["-d"],[rviz_config_path]],
    )
    ld.add_action(rviz2_node)

    

    return ld
