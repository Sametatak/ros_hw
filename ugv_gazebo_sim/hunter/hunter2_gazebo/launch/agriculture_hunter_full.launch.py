import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.descriptions

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. PAKET YOLLARI
    # ---------------------------------------------------------
    pkg_hunter = get_package_share_directory('hunter2_base')
    pkg_agri = get_package_share_directory('cpr_agriculture_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ---------------------------------------------------------
    # 2. MODEL PATH AYARLARI (Mesh'lerin görünmesi için kritik)
    # ---------------------------------------------------------
    hunter_models_path = os.path.join(pkg_hunter, 'models')
    agri_path = os.path.dirname(pkg_agri)
    
    # CPR aksesuarları varsa onları da ekleyelim
    try:
        pkg_accessories = get_package_share_directory('cpr_accessories_gazebo')
        accessories_path = os.path.dirname(pkg_accessories)
    except:
        accessories_path = ""

    # Mevcut yolları koruyarak yenilerini ekle
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    path_set = {hunter_models_path, agri_path}
    if accessories_path:
        path_set.add(accessories_path)
    if existing_model_path:
        for p in existing_model_path.split(':'):
            if p: path_set.add(p)

    set_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=':'.join(list(path_set))
    )

    # ---------------------------------------------------------
    # 3. TANIMLAMALAR (URDF / XACRO)
    # ---------------------------------------------------------
    # Robot Tanımı
    hunter_xacro_file = os.path.join(pkg_hunter, 'urdf', 'hunter2_base_gazebo.xacro')
    hunter_description_content = Command(['xacro ', hunter_xacro_file])

    # Tarım Arazisi Tanımı
    agri_xacro_file = os.path.join(pkg_agri, 'urdf', 'agriculture_geometry.urdf.xacro')
    agri_description_content = Command(['xacro ', agri_xacro_file])

    # ---------------------------------------------------------
    # 4. NODE'LAR VE DOSYALAR
    # ---------------------------------------------------------
    
    # Gazebo Başlatma
    default_world_path = os.path.join(pkg_agri, 'worlds', 'actually_empty_world.world')
    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            "use_sim_time": "true",
            "world": default_world_path,
            "gui": "true",
        }.items(),
    )

    # Tarım Arazisi State Publisher (Remapped)
    agri_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='agriculture_state_publisher',
        parameters=[{'robot_description': agri_description_content}],
        remappings=[('/robot_description', '/agriculture_description')]
    )

    # Robot State Publisher
    hunter_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='hunter_state_publisher',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(hunter_description_content, value_type=str),
            'use_sim_time': True
        }]
    )

    # Araziyi Spawn Et
    spawn_agri = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/agriculture_description', 
            '-entity', 'agriculture_geometry'
        ],
        output='screen'
    )

    # Robotu Spawn Et
    spawn_hunter = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'hunter2',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_hunter, 'rviz/urdf.rviz')],
        output='screen'
    )

    # ---------------------------------------------------------
    # 5. FIRLATMA
    # ---------------------------------------------------------
    return LaunchDescription([
        set_model_path_cmd,
        gazebo_ld,
        agri_state_pub,
        hunter_state_pub,
        spawn_agri,
        spawn_hunter,
        rviz_node
    ])
