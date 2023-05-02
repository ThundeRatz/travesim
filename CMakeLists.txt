cmake_minimum_required(VERSION 3.5)

project(travesim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(gz_msgs REQUIRED)
foreach(dir config launch meshes urdf media worlds models)
  install(DIRECTORY ${dir}
    DESTINATION share/${PROJECT_NAME})
endforeach(dir)

ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/hooks/add_travesim_resource_path.sh.in)

ament_auto_package()
