cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(tutorials_btros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp std_msgs
    behaviortree_cpp_v3
    #behaviortree_ros
    actionlib_msgs
    actionlib
    message_generation
)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} actionlib_msgs )

#add_message_files(
#   FILES
#)

add_service_files(
    FILES
    IsThisGreaterThan.srv
    AddTwo.srv
    CompareValues.srv

)

add_action_files(
    DIRECTORY action
    FILES     Dummy.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

include_directories( ${catkin_INCLUDE_DIRS}
                      include
                   )

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES ${PROJECT_NAME}
     CATKIN_DEPENDS ${ROS_DEPENDENCIES} actionlib_msgs
     DEPENDS
)

######################################################
#EXECUTABLES

## MAIN BT
add_executable(bt_executor src/bt_executor.cpp
                           src/bt_service_node_greaterthan.cpp
                           src/bt_service_node_comparevalues.cpp
                           src/bt_service_node_addtwo.cpp
                           src/bt_action_node_dummy.cpp)
add_dependencies(bt_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bt_executor yaml-cpp ${catkin_LIBRARIES} )
##

# add_executable(mission_planner_bt src/mission_planner_bt_main.cpp)
# add_dependencies(mission_planner_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(mission_planner_bt ${catkin_LIBRARIES} )

add_executable(ros_service_server src/ros_service_server.cpp)
add_dependencies(ros_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_service_server ${catkin_LIBRARIES} )

add_executable(ros_action_server src/ros_action_server.cpp)
add_dependencies(ros_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_action_server ${catkin_LIBRARIES} )


find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

######################################################
# INSTALL

######################################################
# EXAMPLES and TOOLS
