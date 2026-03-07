cmake_minimum_required(VERSION 3.0.2)
project(adapt_mcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# Parallel STL (TBB backend for std::execution::par_unseq).
find_package(TBB QUIET)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adapt_mcl
)

add_library(adapt_mcl
  src/likelihood_field.cpp
  src/motion_model.cpp
  src/soft_em_model.cpp
  src/particle_filter.cpp
)

target_include_directories(adapt_mcl
  PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${EIGEN3_INCLUDE_DIRS}
)

target_compile_options(adapt_mcl PRIVATE -O3 -march=native)

if(TBB_FOUND)
  target_link_libraries(adapt_mcl PUBLIC TBB::tbb)
else()
  # Link stdc++ parallel mode fallback
  target_link_libraries(adapt_mcl PUBLIC tbb)
endif()

install(
  TARGETS adapt_mcl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

# ---- Tests ----
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)

  foreach(t test_likelihood_field test_soft_em_model test_particle_filter)
    catkin_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} adapt_mcl GTest::GTest)
  endforeach()
endif()
