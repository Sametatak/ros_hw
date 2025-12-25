cmake_minimum_required(VERSION 3.8)
project(cpr_orchard_gazebo)

# ROS 2 Build System
find_package(ament_cmake REQUIRED)

# Orijinal dosyadaki klasör listesini koruyarak share dizinine kopyalıyoruz
install(DIRECTORY launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Config dosyasını kopyala
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
