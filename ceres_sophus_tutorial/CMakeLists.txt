cmake_minimum_required(VERSION 3.17)
project(untitled6)

set(CMAKE_CXX_STANDARD 14)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(/usr/local/stow/sophus/include/)
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(ceres_sophus main.cpp)
target_link_libraries(ceres_sophus ${CERES_LIBRARIES} fmt)

#add_executable(test test.cpp)
#target_link_libraries(test ${CERES_LIBRARIES} fmt)