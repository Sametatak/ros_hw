cmake_minimum_required(VERSION 3.8)
project(gazebo_race_modules)

# ROS 2 Build System
find_package(ament_cmake REQUIRED)

# Orijinal dosyadaki klasörleri share dizinine kopyala
install(DIRECTORY docs launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Config dosyasını kopyala
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
