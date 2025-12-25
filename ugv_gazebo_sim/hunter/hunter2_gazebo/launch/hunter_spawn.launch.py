import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import launch_ros.descriptions

def generate_launch_description():
    package_name = 'hunter2_base'
    pkg_share = get_package_share_directory(package_name)
    
    # Modelleme yollarını ekle (Agriculture yollarını bozmadan!)
    gazebo_models_path = os.path.join(pkg_share, 'models')
    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] += ":" + gazebo_models_path
    else:
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Robot verisini yayınla
    urdf_file = os.path.join(pkg_share, 'urdf', 'hunter2_base_gazebo.xacro')
    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='hunter_state_publisher',
            parameters=[{
                'robot_description': launch_ros.descriptions.ParameterValue(robot_description, value_type=str),
                'use_sim_time': True
            }]
        ),

        # Robotu Spawn Et (Z: 0.5 yapıldı ki yere saplanmasın)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'hunter2',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.5', '-Y', '0.0'
            ],
            output='screen'
        ),

        # İstersen RViz'i buradan da açabilirsin
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz/urdf.rviz')]
        )
    ])
