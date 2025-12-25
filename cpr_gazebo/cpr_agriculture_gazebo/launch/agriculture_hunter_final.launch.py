import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    # ---------------------------------------------------------
    # 1. Paket Yolları
    # ---------------------------------------------------------
    pkg_cpr_agriculture = get_package_share_directory('cpr_agriculture_gazebo')
    pkg_cpr_accessories = get_package_share_directory('cpr_accessories_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_hunter_gazebo = get_package_share_directory('hunter2_gazebo') 

    # ---------------------------------------------------------
    # 2. GAZEBO_MODEL_PATH Ayarı (Mesh ve Texture Yolları)
    # ---------------------------------------------------------
    agriculture_path = os.path.dirname(pkg_cpr_agriculture)
    accessories_path = os.path.dirname(pkg_cpr_accessories)
    hunter_path = os.path.dirname(pkg_hunter_gazebo) 

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Tüm yolları birleştirelim
    path_set = {agriculture_path, accessories_path, hunter_path}
    if existing_model_path:
        for p in existing_model_path.split(':'):
            if p: path_set.add(p)

    new_model_path = ':'.join(list(path_set))

    set_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=new_model_path
    )

    # ---------------------------------------------------------
    # 3. Gazebo Başlatma
    # ---------------------------------------------------------
    world_file_name = 'actually_empty_world.world'
    world_path = os.path.join(pkg_cpr_agriculture, 'worlds', world_file_name)

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # ---------------------------------------------------------
    # 4. Tarım Arazisini (Environment) Spawn Etme
    # ---------------------------------------------------------
    agri_xacro_file = os.path.join(pkg_cpr_agriculture, 'urdf', 'agriculture_geometry.urdf.xacro')
    agri_description_content = Command(['xacro ', agri_xacro_file])

    node_agriculture_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='agriculture_state_publisher',
        output='screen',
        parameters=[{'robot_description': agri_description_content}],
        remappings=[('/robot_description', '/agriculture_description')]
    )

    spawn_agriculture = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/agriculture_description', '-entity', 'agriculture_geometry', '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # ---------------------------------------------------------
    # 5. Hunter Robot Spawn
    # ---------------------------------------------------------
    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_hunter_gazebo, 'launch', 'hunter_spawn.launch.py')
        ]),
        launch_arguments={
            'start_x': '0', 'start_y': '0', 'start_z': '0.3', 
            'pub_tf': 'true', 'tf_freq': '100.0'
        }.items()
    )

    # ---------------------------------------------------------
    # 6. RViz2 Başlatma
    # ---------------------------------------------------------
    rviz_config_file = os.path.join(pkg_hunter_gazebo, 'rviz', 'default.rviz')

    # Eğer RViz dosyası yoksa sadece boş açılacak, sorun değil.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        set_model_path_cmd,
        gazebo_simulator,
        node_agriculture_state_publisher,
        spawn_agriculture,
        spawn_car,
        rviz_node
    ])
