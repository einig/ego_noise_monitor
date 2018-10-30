cmake_minimum_required(VERSION 2.8.3)
project(ego_noise_monitor)

add_compile_options(-std=c++11)

set(HEADERS
)

set(SOURCE_FILES
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib ${SOURCE_FILES} ${HEADERS})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})