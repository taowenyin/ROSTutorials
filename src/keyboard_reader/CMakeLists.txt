cmake_minimum_required(VERSION 2.8.3)
project(keyboard_reader)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# To determine keyboard priority, based on active GUI window
find_package(X11 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Key.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES keyboard
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${X11_LIBRARY_DIR})

add_library(check_for_keyboard_priority src/check_for_keyboard_priority.cpp)   
target_link_libraries(check_for_keyboard_priority ${catkin_LIBRARIES} ${X11_LIBRARIES} )   
add_dependencies(check_for_keyboard_priority temoto_generate_messages_cpp)

add_library(keyboard_reader src/keyboard_reader.cpp)
target_link_libraries(keyboard_reader ${catkin_LIBRARIES})
add_dependencies(keyboard_reader ${catkin_EXPORTED_TARGETS})

add_executable(keyboard_event_publisher src/keyboard_event_publisher.cpp)
target_link_libraries(keyboard_event_publisher ${catkin_LIBRARIES} keyboard_reader check_for_keyboard_priority)
add_dependencies(keyboard_event_publisher keyboard_reader_gencpp ${catkin_EXPORTED_TARGETS})
