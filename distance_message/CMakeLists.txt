cmake_minimum_required(VERSION 2.8.3)
project(distance_message)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation nav_msgs message_filters dynamic_reconfigure distance_service)

add_message_files(
 FILES
 Status.msg

 )
 
generate_messages(
   DEPENDENCIES
  std_msgs
 )


generate_dynamic_reconfigure_options(
  cfg/dynamic_distance.cfg
  
)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime nav_msgs message_filters)




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(pub_status src/pub.cpp)
add_dependencies(pub_status ${PROJECT_NAME}_gencfg custom_messages_generate_messages_cpp)
target_link_libraries(pub_status ${catkin_LIBRARIES})


#add_executable(status_sub src/sub.cpp)
#add_dependencies(status_sub custom_messages_generate_messages_cpp)
#target_link_libraries(status_sub ${catkin_LIBRARIES})

