cmake_minimum_required(VERSION 3.5)
project(control_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}  
  include/control_pkg
  include/FORCESNLPsolver/include    
  include/FORCESNLPsolver    
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



add_executable(ctrl_node src/main.cpp src/pure_pursuit.cpp src/ctrl.cpp src/trajectory.cpp src/FORCESNLPsolver_interface.c src/FORCESNLPsolver_model.c)
target_link_libraries(ctrl_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} -ldl -lm ${CMAKE_CURRENT_SOURCE_DIR}/include/FORCESNLPsolver/lib_target/libFORCESNLPsolver.so)

ament_target_dependencies(ctrl_node 
tf2
tf2_geometry_msgs
rclcpp 
std_msgs 
visualization_msgs
std_srvs
geometry_msgs
ackermann_msgs
sensor_msgs
nav_msgs
vesc_msgs)

install(TARGETS
  ctrl_node
  DESTINATION lib/${PROJECT_NAME}  
  )

install(DIRECTORY include/
  DESTINATION include/
)

set(NODE_NAME "ctrl_node")
# install(DIRECTORY include/FORCESNLPsolver/lib/
#   DESTINATION lib/${PROJECT_NAME}/${NODE_NAME}
# )

ament_export_dependencies(
  Eigen3
  rclcpp
  std_msgs
  visualization_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  ackermann_msgs
  nav_msgs
  vesc_msgs
  tf2_geometry_msgs
)





# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
