cmake_minimum_required(VERSION 3.8)
project(cpr_empty_gazebo)

# ROS 2 Build System
find_package(ament_cmake REQUIRED)

# Launch, rviz, urdf ve worlds klasörlerini share dizinine kopyala
install(DIRECTORY launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
