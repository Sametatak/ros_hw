cmake_minimum_required(VERSION 3.5)
project(cpr_agriculture_gazebo)

# ROS 2 Build System
find_package(ament_cmake REQUIRED)

# Eğer ortam dosyalarında xacro/urdf işlemleri varsa bunları da bulması iyi olur
# ancak sadece dosya kopyalayacağımız için zorunlu değil.

# Klasörleri (launch, urdf, worlds, meshes vb.) ROS 2 share klasörüne kopyala
install(DIRECTORY 
  images 
  launch 
  meshes 
  rviz 
  scripts 
  urdf 
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Tekil dosyaları kopyala (model.config gibi)
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
