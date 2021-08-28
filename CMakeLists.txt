cmake_minimum_required(VERSION 2.8.3)
project(ekf)

set(CMAKE_BUILD_TYPE "Debug")
#set C++ version
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekf
#  CATKIN_DEPENDS roscpp rospy std_msgs
DEPENDS EIGEN3
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/include)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(imu_test 
src/imu_test.cpp)
target_link_libraries(imu_test ${catkin_LIBRARIES})

add_executable(imu_px4 
src/imu_px4.cpp)
target_link_libraries(imu_px4 ${catkin_LIBRARIES})

add_executable(opti_transform 
src/opti_transform.cpp)
target_link_libraries(opti_transform ${catkin_LIBRARIES})

add_executable(rotate 
src/rotate.cpp)
target_link_libraries(rotate ${catkin_LIBRARIES})

add_executable(imu_rot 
src/imu_rot.cpp)
target_link_libraries(imu_rot ${catkin_LIBRARIES})

add_executable(magcal 
src/magcal.cpp)
target_link_libraries(magcal ${catkin_LIBRARIES})

add_executable(xsens_rot 
src/xsens_rot.cpp)
target_link_libraries(xsens_rot ${catkin_LIBRARIES})
