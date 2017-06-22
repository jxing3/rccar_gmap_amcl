cmake_minimum_required(VERSION 2.8.3)
project(rc_car)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs 
  nav_msgs 
  tf
  gcop_comm
  rampage_msgs
  std_msgs
  message_generation
  costmap_2d
  geometry_msgs
  nav_core
  pcl_conversions
  pcl_ros
  pluginlib
  visualization_msgs
)
find_package(PCL REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


 add_message_files(
   FILES
	 Car_control.msg
	 ctrl_command.msg
	 PIDGains.msg
 )

add_service_files(
    DIRECTORY srv
    FILES
    MakeNavPlan.srv
    SetCostmap.srv
    nav_msg.srv
    
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
      std_msgs
      geometry_msgs
       nav_msgs
 )

#######################
## Gazebo Standalone ##
#######################

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   LIBRARIES
        navfn
    CATKIN_DEPENDS
        nav_core
        roscpp
        pluginlib
        roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    SYSTEM
    ${PCL_INCLUDE_DIRS}
)

add_library (navfn src/navfn.cpp src/navfn_ros.cpp)
target_link_libraries(navfn
    ${catkin_LIBRARIES}
    )
add_dependencies(navfn ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


add_executable(movewheel5 src/movewheel5.cpp)
target_link_libraries(movewheel5 ${catkin_LIBRARIES})

add_executable(trajtrack src/trajtrack.cpp)
target_link_libraries(trajtrack ${catkin_LIBRARIES})

add_executable(intergcop src/intergcop.cc)
target_link_libraries(intergcop ${catkin_LIBRARIES})

add_executable(intergcop1 src/intergcop1.cc)
target_link_libraries(intergcop1 ${catkin_LIBRARIES})

add_executable(intergcop2 src/intergcop2.cc)
target_link_libraries(intergcop2 ${catkin_LIBRARIES})

add_executable(intergcop3 src/intergcop3.cc)
target_link_libraries(intergcop3 ${catkin_LIBRARIES})

add_executable(velctrl src/velctrl.cc)
target_link_libraries(velctrl ${catkin_LIBRARIES})

add_executable(velctrl1 src/velctrl1.cc)
target_link_libraries(velctrl1 ${catkin_LIBRARIES})

add_executable(velctrl2 src/velctrl2.cc)
target_link_libraries(velctrl2 ${catkin_LIBRARIES})

add_executable(velctrl_test src/velctrl_test.cc)
target_link_libraries(velctrl_test ${catkin_LIBRARIES})

add_executable(destsend src/destsend.cc)
target_link_libraries(destsend ${catkin_LIBRARIES})

add_executable(destsend1 src/destsend1.cc)
target_link_libraries(destsend1 ${catkin_LIBRARIES})


add_executable(navfn_stand src/navfn_stand.cc)
target_link_libraries(navfn_stand navfn)

add_executable(gazebo_laserscan1  src/gazebo_laserscan1.cc)
target_link_libraries(gazebo_laserscan1  ${catkin_LIBRARIES} 
                                        ${GAZEBO_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                        ${PROTOBUF_LIBRARIES})
add_executable(gmappingtf     src/gmappingtf.cc)
target_link_libraries(gmappingtf     ${catkin_LIBRARIES} 
                                        ${GAZEBO_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                        ${PROTOBUF_LIBRARIES})

add_executable(tfgmaperr     src/tfgmaperr.cc)
target_link_libraries(tfgmaperr     ${catkin_LIBRARIES} 
                                        ${GAZEBO_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                        ${PROTOBUF_LIBRARIES})
message("Current dir: ${PROJECT_SOURCE_DIR}")
include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/extern)
add_executable(dsl src/dsl.cc)
target_link_libraries(dsl libdsl.so  ${catkin_LIBRARIES})
add_executable(dsl2 src/dsl2.cc)
target_link_libraries(dsl2 libdsl.so  ${catkin_LIBRARIES})

add_executable(dsl3 src/dsl3.cc)
target_link_libraries(dsl3 libdsl.so  ${catkin_LIBRARIES})

add_executable(odom src/odom.cc)
target_link_libraries(odom ${catkin_LIBRARIES})



## Declare a cpp library
# add_library(beginner_tutorials
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

## Declare a cpp executable
# add_executable(beginner_tutorials_node src/beginner_tutorials_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_tutorials_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beginner_tutorials beginner_tutorials_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
