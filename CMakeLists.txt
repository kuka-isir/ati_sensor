cmake_minimum_required(VERSION 2.8.3)
project(ati_sensor)
set(EXEC_NAME ft_sensor)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  message_generation 
  std_msgs 
  geometry_msgs 
  tf
)

find_package(LibXml2 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS geometry_msgs std_msgs
DEPENDS ${LIBXML2_LIBRARIES}
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME})

###########
## Build ##
###########

include_directories(include 
  ${LIBXML2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${EXEC_NAME}.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${LIBXML2_LIBRARIES}
)
add_executable(${EXEC_NAME} 
  src/${EXEC_NAME}_node.cpp
)

add_executable(${EXEC_NAME}_test
  test/simple_reader.cpp
)

target_link_libraries(${EXEC_NAME}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${EXEC_NAME}_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${EXEC_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
        
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )