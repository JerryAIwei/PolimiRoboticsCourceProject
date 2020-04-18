cmake_minimum_required(VERSION 2.8.3)
project(custom_messages)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation nav_msgs message_filters)

add_message_files(
 FILES
 Status.msg

 )
 
generate_messages(
   DEPENDENCIES
  std_msgs
 )


## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime nav_msgs message_filters)




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(pub_status src/pub.cpp)
add_dependencies(pub_status custom_messages_generate_messages_cpp)
target_link_libraries(pub_status ${catkin_LIBRARIES})


