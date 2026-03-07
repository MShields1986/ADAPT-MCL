cmake_minimum_required(VERSION 3.0.2)
project(lilocbench_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  message_filters
  adapt_mcl
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tum_logger
  CATKIN_DEPENDS
    roscpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
    tf2_ros tf2_geometry_msgs message_filters adapt_mcl
)

# ---- tum_logger library ----
add_library(tum_logger src/tum_logger.cpp)
target_include_directories(tum_logger
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(tum_logger ${catkin_LIBRARIES})

# ---- localizer_node executable ----
add_executable(localizer_node src/localizer_node.cpp)
target_include_directories(localizer_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${catkin_INCLUDE_DIRS}
)
target_link_libraries(localizer_node tum_logger ${catkin_LIBRARIES})
target_compile_options(localizer_node PRIVATE -O3 -march=native)

install(
  TARGETS localizer_node tum_logger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
