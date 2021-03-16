cmake_minimum_required(VERSION 2.8.3)
project(vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS
        rospy
        roscpp
        tf
        tf_conversions
        std_msgs
        geometry_msgs
        sensor_msgs
        REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# 	plane.msg
# 	planes.msg
#   coordinate.msg
#   coordinates.msg
#   pointCloudVision.msg
#   pointCloudsVision.msg
# )

## Generate services in the 'srv' folder
## add_service_files(
##   FILES
##   save_image.srv
##   depth.srv
##   robot_state.srv
##   pcb.srv
##   ptw.srv
##   pls.srv
##   NN.srv
## )

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    message_runtime
    cv_bridge
)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

catkin_install_python(PROGRAMS src/vision/pythonClasses/acquireData.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/vision/main.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/vision/pixel_to_world.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/vision/generateDataset.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


