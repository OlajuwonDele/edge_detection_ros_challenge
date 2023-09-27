cmake_minimum_required(VERSION 3.0.2)
project(edge_detection)

## Set C++ Standard to C++ 14, necessary to use pcl package
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  cv_bridge
  roscpp
  message_generation
  sensor_msgs 
  image_transport  
  pcl_ros
  visualization_msgs
  tf2_ros
)

################################################
## Declare ROS messages, services, and actions ##
################################################

# Add message and service files if needed
# add_message_files(
#   FILES
#   edge_detect_msg.msg
# )

add_service_files(
  FILES
  edge_detect_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs  # Add sensor_msgs as a message dependency
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs cv_bridge roscpp message_runtime sensor_msgs image_transport pcl_ros  # Add sensor_msgs, image_transport, and pcl_ros
)

###########
## Build ##
###########

## Find OpenCV 4 to be compatible with ROS Noetic
find_package(OpenCV 4 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include  # Access to header files in the include directory
)

# List source files explicitly
set(SOURCE_FILES
  src/EdgeDetector.cpp
)

# Define your library target
add_library(${PROJECT_NAME}_lib SHARED ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Build executables for the nodes
add_executable(edge_detection_publisher src/edge_detection_publisher.cpp)
target_link_libraries(edge_detection_publisher ${PROJECT_NAME}_lib)

add_executable(edge_detection_service src/edge_detection_service.cpp)
target_link_libraries(edge_detection_service ${PROJECT_NAME}_lib)

add_executable(edge_detection_client src/edge_detection_client.cpp)
target_link_libraries(edge_detection_client ${PROJECT_NAME}_lib)

## Specify the output directory for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

##Testing
# add_definitions(-DEDGE_DETECTOR_TEST)
