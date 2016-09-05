cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  visp_bridge
  cv_bridge
)

find_package(OpenCV 2.4.8 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############
install(DIRECTORY launch urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


# Had to use another name to distinguish from the "test_visp" repository
add_executable(visp-tracker src/template-tracker.cpp src/detector/landing_mark_detection.cpp)
target_link_libraries(visp-tracker ${catkin_LIBRARIES})
target_link_libraries(visp-tracker ${OpenCV_LIBRARIES})
  
#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
#  roslaunch_add_file_check(launch/task.launch)
endif()

