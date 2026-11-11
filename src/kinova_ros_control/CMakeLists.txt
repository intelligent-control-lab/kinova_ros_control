cmake_minimum_required(VERSION 3.10)
project(kinova_ros_control VERSION 1.0.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs std_srvs geometry_msgs sensor_msgs robot_controller
)
find_package(Eigen3 3.3.7 REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(CATKIN_DEPENDS
  roscpp std_msgs std_srvs geometry_msgs sensor_msgs robot_controller
)

add_executable(kinova_bringup src/kinova_bringup.cpp src/kinova_bridge.cpp)
target_compile_features(kinova_bringup PRIVATE cxx_std_11)
target_compile_definitions(kinova_bringup PRIVATE _OS_UNIX)
target_include_directories(kinova_bringup PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(kinova_bringup PRIVATE
  ${catkin_LIBRARIES} Eigen3::Eigen spdlog::spdlog
)
add_dependencies(kinova_bringup ${catkin_EXPORTED_TARGETS})

install(TARGETS kinova_bringup
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
