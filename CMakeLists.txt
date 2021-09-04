cmake_minimum_required(VERSION 3.5)
project(multi_robot_state_publisher)

add_definitions(
  -std=c++17
  -W
  -Wall
  -Wextra
  -Wno-ignored-attributes
  -Wno-int-in-bool-context
  )

find_package(orocos_kdl REQUIRED)
find_package(
  catkin
  REQUIRED
  COMPONENTS roscpp
             rosconsole
             rostime
             tf
             tf2_ros
             tf2_kdl
             kdl_parser
  )
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS roscpp
          rosconsole
          rostime
          tf2_ros
          tf2_kdl
          kdl_parser
          orocos_kdl
          urdfdom_headers
  )

include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  )
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(
  ${PROJECT_NAME}
  src/joint_state_listener.cpp
  src/robot_state_publisher.cpp
  )
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  )
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(${PROJECT_NAME}_node src/multi_robot_state_publisher_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${orocos_kdl_LIBRARIES}
  )

# TODO(RWS): Update the tests for the refactor.

# add_subdirectory(test)

install(
  TARGETS ${PROJECT_NAME}
          multi_robot_state_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
