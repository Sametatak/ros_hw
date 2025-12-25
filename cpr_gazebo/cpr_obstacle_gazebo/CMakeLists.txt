cmake_minimum_required(VERSION 3.8)
project(cpr_obstacle_gazebo)

# ROS 2 Build System
find_package(ament_cmake REQUIRED)

# Klasörleri ROS 2 share dizinine kopyala
install(
  DIRECTORY launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Config dosyasını kopyala
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
