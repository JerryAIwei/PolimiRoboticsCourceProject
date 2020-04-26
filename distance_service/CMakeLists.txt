cmake_minimum_required(VERSION 2.8.3)
project(distance_service)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation )

add_service_files(
  FILES
  ComputeDistance.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)




## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(compute_distance src/compute_distance.cpp)
target_link_libraries(compute_distance ${catkin_LIBRARIES})
add_dependencies(compute_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

