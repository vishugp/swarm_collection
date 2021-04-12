cmake_minimum_required(VERSION 3.0.2)
project(swarm_collection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  geometry_msgs
  joy
  mav_msgs
  roscpp
  rospy
  rotors_comm
  rotors_control
  rotors_description
  rotors_gazebo
  rotors_gazebo_plugins
  rotors_joy_interface
  gazebo REQUIRED
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   gazebo_msgs#   geometry_msgs#   mav_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swarm_collection
  CATKIN_DEPENDS gazebo_msgs gazebo_plugins geometry_msgs joy mav_msgs roscpp rospy rotors_comm rotors_control rotors_description rotors_gazebo rotors_gazebo_plugins rotors_joy_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/swarm_collection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/swarm_collection_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
 src/gripper_control.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
 src/gripper_control_test.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_swarm_collection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})





add_executable(square src/square.cpp)
target_link_libraries(square ${catkin_LIBRARIES})
add_dependencies(square ${catkin_EXPORTED_TARGETS})

add_executable(X src/X.cpp)
target_link_libraries(X ${catkin_LIBRARIES})
add_dependencies(X ${catkin_EXPORTED_TARGETS})


add_executable(straight src/straight.cpp)
target_link_libraries(straight ${catkin_LIBRARIES})
add_dependencies(straight ${catkin_EXPORTED_TARGETS})


add_executable(straightnew src/straightnew.cpp)
target_link_libraries(straightnew ${catkin_LIBRARIES})
add_dependencies(straightnew ${catkin_EXPORTED_TARGETS})



add_executable(straightnewnew src/straightnewnew.cpp)
target_link_libraries(straightnewnew ${catkin_LIBRARIES})
add_dependencies(straightnewnew ${catkin_EXPORTED_TARGETS})

add_executable(root_node src/root_node.cpp)
target_link_libraries(root_node ${catkin_LIBRARIES})
add_dependencies(root_node ${catkin_EXPORTED_TARGETS})


add_executable(extra_square src/extra_square.cpp)
target_link_libraries(extra_square ${catkin_LIBRARIES})
add_dependencies(extra_square ${catkin_EXPORTED_TARGETS})

add_executable(extra_root_node src/extra_root_node.cpp)
target_link_libraries(extra_root_node ${catkin_LIBRARIES})
add_dependencies(extra_root_node ${catkin_EXPORTED_TARGETS})




add_executable(interzone src/interzone.cpp)
target_link_libraries(interzone ${catkin_LIBRARIES})
add_dependencies(interzone ${catkin_EXPORTED_TARGETS})

add_executable(interzone_rootnode src/interzone_rootnode.cpp)
target_link_libraries(interzone_rootnode ${catkin_LIBRARIES})
add_dependencies(interzone_rootnode ${catkin_EXPORTED_TARGETS})





add_executable(mav_hover src/mav_hover.cpp)
target_link_libraries(mav_hover ${catkin_LIBRARIES})
add_dependencies(mav_hover ${catkin_EXPORTED_TARGETS})


add_executable(extra_straight src/extra_straight.cpp)
target_link_libraries(extra_straight ${catkin_LIBRARIES})
add_dependencies(extra_straight ${catkin_EXPORTED_TARGETS})







add_executable(gripper_square_extra src/gripper_square_extra.cpp)
target_link_libraries(gripper_square_extra ${catkin_LIBRARIES})
add_dependencies(gripper_square_extra ${catkin_EXPORTED_TARGETS})

foreach(dir launch models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


#install(TARGETS square X straight straightnewnew root_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)



##24 120 130
