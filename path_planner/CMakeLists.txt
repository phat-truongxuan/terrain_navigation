# Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.5)
project(path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AllInfo.msg" DEPENDENCIES builtin_interfaces
 )

 rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


set(dependencies
rclcpp
geometry_msgs
visualization_msgs
std_msgs
rviz_visual_tools
tf2
# tf2_ros
# tf2_eigen
)

include_directories(include)

# Build
add_library(utils SHARED src/utils.cpp)
ament_target_dependencies(utils)

add_library(map_handler SHARED src/map_handler.cpp)
ament_target_dependencies(map_handler ${dependencies})

add_library(default_planner SHARED src/default_planner.cpp)
target_link_libraries(default_planner map_handler utils)
target_link_libraries(default_planner "${cpp_typesupport_target}") 
ament_target_dependencies(default_planner ${dependencies})

add_library(astar_planner SHARED src/astar_planner.cpp)
target_link_libraries(astar_planner default_planner map_handler utils)
target_link_libraries(astar_planner "${cpp_typesupport_target}") 
ament_target_dependencies(astar_planner ${dependencies})

add_library(dstar_lite_planner SHARED src/dstar_lite_planner.cpp)
target_link_libraries(dstar_lite_planner default_planner map_handler utils)
ament_target_dependencies(dstar_lite_planner ${dependencies})



add_library(navigation_node SHARED src/navigation_node.cpp)
target_link_libraries(navigation_node map_handler default_planner astar_planner dstar_lite_planner utils "${cpp_typesupport_target}")
ament_target_dependencies(navigation_node ${dependencies})


# add_executable(dstar_main src/dstar_lite_main.cpp)
# target_link_libraries(dstar_main dstar_lite_planner map_handler)
# ament_target_dependencies(dstar_main ${dependencies})

add_executable(nav_main src/nav_node_main.cpp)
target_link_libraries(nav_main map_handler utils default_planner dstar_lite_planner astar_planner navigation_node)
ament_target_dependencies(nav_main ${dependencies})

add_executable(nav_test src/nav_node_test.cpp)
target_link_libraries(nav_test map_handler utils default_planner dstar_lite_planner astar_planner navigation_node)
ament_target_dependencies(nav_test ${dependencies})

# Install


install(TARGETS default_planner astar_planner dstar_lite_planner map_handler utils navigation_node nav_main nav_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(TARGETS
  map_handler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/map_handler)


install(DIRECTORY include/
  DESTINATION include/
)

# install(
#   PROGRAMS
#     scripts/my_script.py
#   DESTINATION
#     lib/${PROJECT_NAME}
# )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Test
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Macro for ament package
ament_export_include_directories(include)
ament_export_libraries(astar_planner map_handler)
ament_export_dependencies(${dependencies})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
