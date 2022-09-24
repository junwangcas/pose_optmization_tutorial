cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(Eigen3 REQUIRED)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
#add_executable(visualize_pose src/visualize_pose.cpp)
#target_link_libraries(visualize_pose ${catkin_LIBRARIES})

add_executable(quaternion_test src/quaternion_test.cpp src/quaternion_test_chapters.cpp)
target_link_libraries(quaternion_test ${catkin_LIBRARIES})


add_executable(lie_group_test lie_group/visualization/main.cpp lie_group/visualization/camera_pose_vis.cpp)
target_link_libraries(lie_group_test ${catkin_LIBRARIES})



catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)