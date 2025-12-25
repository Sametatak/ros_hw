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
    # 1. Paket ve Dosya Yolları
    # ---------------------------------------------------------
    pkg_hunter_base = get_package_share_directory('hunter2_base')
    pkg_cpr_agriculture = get_package_share_directory('cpr_agriculture_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Hunter Robot Dosyaları
    hunter_urdf_path = os.path.join(pkg_hunter_base, 'urdf', 'hunter2_base_gazebo.xacro')
    
    # Tarım Dünyası Dosyaları
    world_path = os.path.join(pkg_cpr_agriculture, 'worlds', 'actually_empty_world.world')
    agri_xacro_path = os.path.join(pkg_cpr_agriculture, 'urdf', 'agriculture_geometry.urdf.xacro')

    # ---------------------------------------------------------
    # 2. GAZEBO_MODEL_PATH Ayarı
    # ---------------------------------------------------------
    # Hem Hunter modellerini hem de Tarım modellerini Gazebo'ya tanıtıyoruz
    hunter_models = os.path.join(pkg_hunter_base, 'models')
    agri_models = os.path.dirname(pkg_cpr_agriculture) # CPR paketleri genelde üst dizinden mesh çeker
    
    existing_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{hunter_models}:{agri_models}:{existing_path}"

    set_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=new_model_path
    )

    # ---------------------------------------------------------
    # 3. Robot ve Ortam Tanımları (Xacro Processing)
    # ---------------------------------------------------------
    # Hunter Robot State Publisher
    hunter_description = Command(['xacro ', hunter_urdf_path])
    
    start_hunter_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='hunter_state_publisher',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(hunter_description, value_type=str),
            'use_sim_time': True
        }]
    )

    # Tarım Arazisi State Publisher (Remapping ile robot_description'dan ayırıyoruz)
    agri_description = Command(['xacro ', agri_xacro_path])
    
    start_agri_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='agriculture_state_publisher',
        parameters=[{'robot_description': agri_description}],
        remappings=[('/robot_description', '/agriculture_description')]
    )

    # ---------------------------------------------------------
    # 4. Gazebo ve Spawn İşlemleri
    # ---------------------------------------------------------
    # Gazebo'yu başlat
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # Tarım Arazisini Dünyaya Ekle (Environment Spawner)
    spawn_agri_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/agriculture_description', '-entity', 'agriculture_geometry'],
        output='screen'
    )

    # Hunter Robotu Dünyaya Ekle (Robot Spawner)
    # Z yüksekliği 0.3 yapıldı ki araziye saplanmasın
    spawn_hunter_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'hunter2', 
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.3', '-Y', '0.0'
        ],
        output='screen'
    )

    # Senin dosandaki Ackermann Kontrolcüsü (Eğer config dosyaların hazırsa)
    load_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gazebo_ros_ackermann_drive"],
    )

    # ---------------------------------------------------------
    # 5. Launch Description Oluşturma
    # ---------------------------------------------------------
    ld = LaunchDescription()

    # Aksiyonları sırayla ekle
    ld.add_action(set_model_path_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_agri_state_publisher)
    ld.add_action(start_hunter_state_publisher)
    ld.add_action(spawn_agri_cmd)
    ld.add_action(spawn_hunter_cmd)
    # ld.add_action(load_joint_controller) # Kontrolcülerin hazırsa bunu aktif edebilirsin

    return ld
