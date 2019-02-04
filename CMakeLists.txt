cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_shaft_ascend)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg mavros)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ros_erle_shaft_ascend src/main.cpp )
target_link_libraries(ros_erle_shaft_ascend ${catkin_LIBRARIES})
add_dependencies(ros_erle_shaft_ascend ros_erle_shaft_ascend_generate_messages_cpp)

install(
	TARGETS ros_erle_shaft_ascend
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(main_sense
  src/main_sense.cpp
)
add_dependencies(main_sense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_sense
  ${catkin_LIBRARIES}
)

add_executable(iter_1
  src/iter_1.cpp
)
add_dependencies(iter_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iter_1
  ${catkin_LIBRARIES}
)