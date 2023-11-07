cmake_minimum_required(VERSION 3.0.2)
project(my_turtlebot_launch)

set(pcl_conversions_DIR /home/thy/depend/catkin_depends/devel_isolated/pcl_conversions/share/pcl_conversions/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  tf
  pcl_conversions
  std_msgs
  sensor_msgs
  gazebo_msgs
  nav_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Time.msg
  Elipse.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_turtlebot_launch
 CATKIN_DEPENDS roscpp rospy message_runtime sensor_msgs 
 DEPENDS PCL GTSAM
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_executable(imu_pre src/imu_pre.cpp)
target_link_libraries(imu_pre
  ${catkin_LIBRARIES}
  gtsam
)

add_executable(imu_lidar src/imu_lidar.cpp)
target_link_libraries(imu_lidar
  ${catkin_LIBRARIES}
  gtsam
)

add_executable(imu_lidar_kf src/imu_lidar_kf.cpp)
target_link_libraries(imu_lidar_kf
  ${catkin_LIBRARIES}
  gtsam
)


## Specify libraries to link a library or executable target against
add_executable(path_br src/path_br.cpp)
target_link_libraries(path_br
  ${catkin_LIBRARIES}
)

add_executable(path_odom src/path_odom.cpp)
target_link_libraries(path_odom
  ${catkin_LIBRARIES}
)

add_executable(imu_pub_txt src/imu_pub_txt.cpp)
target_link_libraries(imu_pub_txt
  ${catkin_LIBRARIES}
)

# add_executable(plane_fit src/plane_fit.cpp)
# target_link_libraries(plane_fit
#   ${PCL_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

add_executable(cylinder_fit src/cylinder_fit.cpp)
target_link_libraries(cylinder_fit
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(pcd_rviz src/pcd_rviz.cpp)
target_link_libraries(pcd_rviz
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(lidar_KF src/lidar_KF.cpp)
target_link_libraries(lidar_KF
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  gtsam
)

# add_executable(test_pcl src/test_pcl.cpp)
# target_link_libraries(test_pcl
#   ${catkin_LIBRARIES}
# )
# add_executable(yaw_cal src/yaw_cal.cpp)
# target_link_libraries(yaw_cal
#   ${catkin_LIBRARIES}
# )
