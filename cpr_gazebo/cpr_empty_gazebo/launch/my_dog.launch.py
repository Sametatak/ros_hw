import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Konfigürasyon Argümanları (Senin dosyadaki default değerler)
    # x=0.0, y=-10.0, z=2.0 (z'yi biraz yüksek tuttum ki yere gömülmesin)
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='-10.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.5') # 2.0 çok yüksek olabilir, 0.5 güvenlidir
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    # 2. Dosya Yolları
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_warthog_description = get_package_share_directory('my_quadruped_control')

    # Robotun URDF/Xacro Dosyası
    xacro_file = os.path.join(pkg_warthog_description, 'urdf', 'warthog.urdf.xacro')

    # 3. Gazebo'yu Başlat (Empty World)
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # verbose=true diyerek hata ayıklamayı kolaylaştırabilirsin
        launch_arguments={'verbose': 'true'}.items() 
    )

    # 4. Robot State Publisher (Xacro'yu URDF'e çevirip yayınlar)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # 5. Robotu Gazebo'ya Spawn Etme (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'warthog',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        start_gazebo,
        robot_state_publisher,
        spawn_entity
    ])
