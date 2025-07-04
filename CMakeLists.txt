cmake_minimum_required(VERSION 3.8)
project(ugv_casualty_inspection_list_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(base_node_msgs REQUIRED)
find_package(geographic_utils REQUIRED)


find_package(triage_task_allocation_interface REQUIRED)



set(LIB_NAME ${PROJECT_NAME}_impl)

add_library(${LIB_NAME}
  src/allocation_manager.cpp)


target_include_directories(${LIB_NAME} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}> )

ament_target_dependencies(${LIB_NAME}
  PUBLIC
  rclcpp
  std_msgs
  std_srvs

  base_node_msgs
  geographic_utils

  triage_task_allocation_interface
)


install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${LIB_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  std_msgs 
  std_srvs 
)

add_executable(ugv_casualty_inspection_list_manager_node
  src/allocation_manager_node.cpp)


target_link_libraries(ugv_casualty_inspection_list_manager_node
  PUBLIC
  rclcpp::rclcpp
  ${LIB_NAME}
  )

target_compile_features(ugv_casualty_inspection_list_manager_node PUBLIC cxx_std_17)  # Require C++17

install(TARGETS ugv_casualty_inspection_list_manager_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
