cmake_minimum_required(VERSION 3.8)
project(hexarover_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hexarover_description REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
install(FILES
      config/slam_toolbox.yaml
      config/rf2o.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY launch rviz 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
