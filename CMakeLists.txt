cmake_minimum_required(VERSION 3.0.2)
project(dslr_camera_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  image_transport
  cv_bridge
)

find_package(OpenCV)

FIND_PACKAGE(PkgConfig  REQUIRED)
PKG_CHECK_MODULES(GPHOTO2  libgphoto2>=2.4.6 REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
  Capture.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS GPHOTO2
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/dslr_camera_ros_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${GPHOTO2_LIBRARIES} ${OpenCV_LIBRARIES}
)

