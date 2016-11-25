cmake_minimum_required(VERSION 2.8.3)
project(jaguar_base)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(catkin REQUIRED COMPONENTS
    controller_manager hardware_interface jaguar_msgs roslaunch roslint roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS hardware_interface jaguar_msgs roscpp sensor_msgs
DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(jaguar_node
  src/jaguar_base.cpp
  src/jaguar_hardware.cpp
  src/drrobot_wrapper.cpp
#  src/jaguar_diagnostics.cpp
  src/DrRobotMotionSensorDriver.cpp)
target_link_libraries(jaguar_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(jaguar_node jaguar_msgs_gencpp)

roslint_cpp(
  src/jaguar_base.cpp
  src/jaguar_hardware.cpp
  #src/jaguar_diagnostics.cpp
  src/drrobot_wrapper.cpp
  src/DrRobotMotionSensorDriver.cpp
  #include/jaguar_base/jaguar_diagnostics.hpp
  include/jaguar_base/drrobot_wrapper.hpp
  include/jaguar_base/jaguar_hardware.hpp
  include/jaguar_base/DrRobotMotionSensorDriver.hpp
)

#install(TARGETS horizon_legacy jaguar_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY launch config
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})