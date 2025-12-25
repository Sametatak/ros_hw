cmake_minimum_required(VERSION 3.8)
project(cpr_accessories_gazebo)

# ROS 2 Build System
find_package(ament_cmake REQUIRED)

# Dosyaları ROS 2 share klasörüne kopyala
install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Tekil config dosyasını kopyala
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
