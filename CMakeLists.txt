cmake_minimum_required(VERSION 2.8.2)
project(ati_sensor)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
  geometry_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)

catkin_package(
    CATKIN_DEPENDS geometry_msgs std_msgs
    INCLUDE_DIRS include
    LIBRARIES ati_sensor
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(ati_sensor src/ft_sensor.cpp)

find_package(Xenomai QUIET)
# Allows to build for Xenomai + RTnet
if(XENOMAI_FOUND OR (DEFINED $ENV{OROCOS_TARGET} AND $ENV{OROCOS_TARGET} EQUAL "xenomai"))
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)
    message("[${PROJECT_NAME}] Building ATI FT/Sensor for Xenomai ${XENOMAI_VERSION} + RTnet")
    if(${XENOMAI_VERSION_MAJOR} EQUAL 3)
        find_package(XenomaiPosix REQUIRED)
    endif()
    if(${XENOMAI_VERSION_MAJOR} EQUAL 2)
        find_package(RTnet REQUIRED)
        include_directories(${XENOMAI_INCLUDE_DIRS} ${RTNET_INCLUDE_DIRS})
        target_link_libraries(ati_sensor rtdm ${XENOMAI_LIBRARIES})
    endif()
    set_target_properties(ati_sensor PROPERTIES COMPILE_FLAGS -DXENOMAI_VERSION_MAJOR=${XENOMAI_VERSION_MAJOR})
else()
    find_package(LibXml2 REQUIRED)
    include_directories(${LIBXML2_INCLUDE_DIR})
endif()

target_link_libraries(ati_sensor ${LIBXML2_LIBRARIES} ${XENOMAI_POSIX_LIBRAIRIES})

add_executable(ft_sensor_node src/ft_sensor_node.cpp)
target_link_libraries(ft_sensor_node ati_sensor ${catkin_LIBRARIES})

add_executable(simple_reader_test test/simple_reader.cpp)
target_link_libraries(simple_reader_test ati_sensor)

add_executable(gauge_bias_tester  test/test_gauge_bias.cpp)
target_link_libraries(gauge_bias_tester ati_sensor)

install(TARGETS ati_sensor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS ft_sensor_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
