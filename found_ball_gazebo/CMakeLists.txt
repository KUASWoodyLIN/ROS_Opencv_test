cmake_minimum_required(VERSION 2.8.3)
project(found_ball_gazebo)

#set(CMAKE_PREFIX_PATH "/usr/local/lib")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  mavros_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/local/include/)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Ballinfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    roscpp
    image_transport
    cv_bridge
    mavros_msgs
    message_runtime
    #LIBRARIES ${PROJECT_NAME}
)

### Build ball_tracking ###
add_executable(ball_tracking
  src/ball_tracking.cpp
)
target_link_libraries(ball_tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(ball_tracking ball_tracking)

### Build image_publisher ###
add_executable(image_publisher
  src/image_publisher.cpp
)
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}  ${OpenCV_LIBS} ${OpenCV_LIBRARIES}  /usr/local/lib/libaruco.so)
add_dependencies(image_publisher image_publisher)

### Build copter_control ###
add_executable(copter_control
  src/copter_control.cpp
)
target_link_libraries(copter_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}  ${OpenCV_LIBS} ${OpenCV_LIBRARIES} /usr/local/lib/libaruco.so )
add_dependencies(copter_control copter_control)



