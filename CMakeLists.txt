cmake_minimum_required(VERSION VERSION 3.5.1)
project(gazebo-plugin-autonomous-actor)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  gazebo_person_detection

)

find_package(gazebo REQUIRED)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS}  ${catkin_LIBRARY_DIRS} ${tiago_public_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} )

catkin_package(
  DEPENDS
    roscpp
    rospy
    std_msgs
    
)

add_library(AutonomousActorPlugin SHARED src/AutonomousActorPlugin.cc)
target_link_libraries(AutonomousActorPlugin ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES} )

# Debug Version (Note to add library before target compile definition)
add_library(AutonomousActorPluginDebug SHARED src/AutonomousActorPlugin.cc)
target_compile_definitions(AutonomousActorPluginDebug PRIVATE DEBUG_)
target_link_libraries(AutonomousActorPluginDebug ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(CollisionActorPlugin SHARED src/CollisionActorPlugin.cc)
target_link_libraries(CollisionActorPlugin ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES} )

# Debug Version (Note to add library before target compile definition)
add_library(CollisionActorPluginDebug SHARED src/CollisionActorPlugin.cc)
target_compile_definitions(CollisionActorPluginDebug PRIVATE DEBUG_)
target_link_libraries(CollisionActorPluginDebug ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(AttachModelPlugin SHARED src/AttachModelPlugin.cc)
target_link_libraries(AttachModelPlugin ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES} )




