cmake_minimum_required(VERSION 2.8.3)
project(VideoCapture_cam)

#set(CMAKE_PREFIX_PATH "/usr/local/lib")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
 )

find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
)


###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/VideoCapture_cam.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})

