cmake_minimum_required(VERSION 3.8)
project(cpr_inspection_gazebo)

# ROS 2 Build System
find_package(ament_cmake REQUIRED)

# Tüm asset klasörlerini kopyala
install(DIRECTORY images launch meshes models rviz scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Config dosyasını kopyala
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
