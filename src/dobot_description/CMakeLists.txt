cmake_minimum_required(VERSION 3.8)
project(dobot_description)

find_package(ament_cmake REQUIRED)

# Install all directories
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
