import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    # ---------------------------------------------------------
    # 1. PAKET VE YOL AYARLARI
    # ---------------------------------------------------------
    pkg_cpr_agriculture = get_package_share_directory('cpr_agriculture_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Mesh ve Texture dosyalarının bulunması için GAZEBO_MODEL_PATH ayarı
    agriculture_path = os.path.dirname(pkg_cpr_agriculture)
    
    # CPR paketleri genelde accessories paketine de ihtiyaç duyar
    try:
        pkg_cpr_accessories = get_package_share_directory('cpr_accessories_gazebo')
        accessories_path = os.path.dirname(pkg_cpr_accessories)
    except:
        accessories_path = ""

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    path_set = {agriculture_path}
    if accessories_path:
        path_set.add(accessories_path)
    if existing_model_path:
        for p in existing_model_path.split(':'):
            if p: path_set.add(p)

    set_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=':'.join(list(path_set))
    )

    # Dünya Dosyası Yolu (Orijinal ROS 1 dosyasındaki aslında_boş_dünya)
    default_world_path = os.path.join(pkg_cpr_agriculture, 'worlds', 'actually_empty_world.world')

    # ---------------------------------------------------------
    # 2. AGRICULTURE GEOMETRİSİNİ (ÇEVRE) HAZIRLAMA
    # ---------------------------------------------------------
    agri_xacro_file = os.path.join(pkg_cpr_agriculture, 'urdf', 'agriculture_geometry.urdf.xacro')
    agri_description_content = Command(['xacro ', agri_xacro_file])

    # Çevreyi URDF olarak yayınlayan publisher (Remapping ile robot_description'dan ayırıyoruz)
    agriculture_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='agriculture_state_publisher',
        output='screen',
        parameters=[{'robot_description': agri_description_content}],
        remappings=[('/robot_description', '/agriculture_description')]
    )

    # Çevreyi Gazebo'ya Spawn etme
    spawn_agriculture_geom = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/agriculture_description', 
            '-entity', 'agriculture_geometry', 
            '-x', '0', '-y', '0', '-z', '0', '-Y', '0'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # 3. ARGÜMANLAR VE GAZEBO BAŞLATMA
    # ---------------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true"
    )

    # Gazebo Simülatörünü Başlat
    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    # ---------------------------------------------------------
    # 4. ÇIKTI (LaunchDescription)
    # ---------------------------------------------------------
    return LaunchDescription(
        [
            set_model_path_cmd,
            declare_use_sim_time,
            declare_gazebo_world,
            declare_gui,
            agriculture_state_publisher, # Ortam bilgisini yayınla
            gazebo_ld,                  # Boş dünyayı aç
            spawn_agriculture_geom,      # Ortamı boş dünyaya yerleştir
        ]
    )
