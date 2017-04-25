cmake_minimum_required(VERSION 2.8.3)
project(ati_sensor)

set(CMAKE_BUILD_TYPE Release)

set(library_name ati_sensor)
set(ros_exec_name ft_sensor)
set(standalone_exec_name ati_sensor_standalone_test)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_srvs
  std_msgs
  geometry_msgs
)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS geometry_msgs std_msgs
    INCLUDE_DIRS include
    LIBRARIES ${library_name}
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

#### Main library

add_library(${library_name}
    src/ft_sensor.cpp
)

# Allows to build for Xenomai + RTnet
find_package(Xenomai QUIET)

if(XENOMAI_FOUND OR (DEFINED $ENV{OROCOS_TARGET} AND $ENV{OROCOS_TARGET} EQUAL "xenomai"))
  add_definitions(-DXENOMAI_VERSION_MAJOR=${XENOMAI_VERSION_MAJOR})
  if(${XENOMAI_VERSION_MAJOR} EQUAL 2)
      find_package(RTnet REQUIRED)
      include_directories(${RTNET_INCLUDE_DIRS})
      target_link_libraries(${library_name} rtdm)
  endif()
  include_directories(${XENOMAI_INCLUDE_DIR})
  message("Building ATI FT/Sensor with Xenomai + RTNET")
else()
  find_package(LibXml2 REQUIRED)
  include_directories(${LIBXML2_INCLUDE_DIR})
endif()

target_link_libraries(${library_name} ${LIBXML2_LIBRARIES} ${XENOMAI_LIBRARIES})


#### ROS wrapper
add_executable(${ros_exec_name}
  src/ft_sensor_node.cpp
)
target_link_libraries(${ros_exec_name}
  ${library_name}
  ${catkin_LIBRARIES}
  ${XENOMAI_LIBRARIES}
)

#### Simple tester (no ROS)

add_executable(${standalone_exec_name}
  test/simple_reader.cpp
)
target_link_libraries(${standalone_exec_name}
  ${library_name}
  ${XENOMAI_LIBRARIES}
)

add_executable(gauge_bias_tester
  test/test_gauge_bias.cpp
)
target_link_libraries(gauge_bias_tester
  ${library_name}
)



#############
## Install ##
#############

install(TARGETS ${library_name} ${standalone_exec_name}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
        
install(TARGETS ${ros_exec_name}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
              

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
