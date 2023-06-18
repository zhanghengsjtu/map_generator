cmake_minimum_required(VERSION 3.0.2)
project(mm_local_motion_planning)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation moveit_core moveit_ros_planning_interface)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    agv_distance_detection
    agv_collision_map
  CATKIN_DEPENDS message_runtime 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${YAML_CPP_LIBRARIES})

add_library(agv_distance_detection include/mm_local_motion_planning/agv_distance_detection.cpp)
target_link_libraries(agv_distance_detection ${catkin_LIBRARIES})

add_library(agv_collision_map include/mm_local_motion_planning/agv_collision_map.cpp)
target_link_libraries(agv_collision_map ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} agv_distance_detection)

add_executable(calculateCollisionMap src/calculateCollisionMap.cpp)
add_dependencies(calculateCollisionMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calculateCollisionMap ${catkin_LIBRARIES} agv_collision_map)
