################################################################################
#
# CMake script for finding RTNet.
# If the optional RTNET_ROOT_DIR environment variable exists, header files and
# libraries will be searched in the RTNET_ROOT_DIR/include and RTNET_ROOT_DIR/lib
# directories, respectively. Otherwise the default CMake search process will be
# used.
#
# This script creates the following variables:
#  RTNET_FOUND: Boolean that indicates if the package was found
#  RTNET_INCLUDE_DIRS: Paths to the necessary header files
#  RTNET_LIBRARIES: Package libraries
#
################################################################################

set(RTNET_FOUND TRUE)

# Get hint from environment variable (if any)
if(NOT $ENV{RTNET_ROOT_DIR} STREQUAL "")
	set(RTNET_ROOT_DIR $ENV{RTNET_ROOT_DIR} CACHE PATH "RTNet base directory location (optional, used for nonstandard installation paths)" FORCE)
	mark_as_advanced(RTNET_ROOT_DIR)
endif()

# Header files to find
set(header_NAME    rtnet.h)

# Find headers and libraries
if(RTNET_ROOT_DIR)
	# Use location specified by environment variable
	find_path(RTNET_INCLUDE_DIR		NAMES ${header_NAME}			PATHS ${RTNET_ROOT_DIR}/include PATH_SUFFIXES rtnet NO_DEFAULT_PATH)
else(RTNET_ROOT_DIR)
	# Use default CMake search process (plus hard coded path...)
	find_path(RTNET_INCLUDE_DIR		NAMES ${header_NAME}			PATHS /usr/local/rtnet/include /usr/include/xenomai PATH_SUFFIXES rtnet )
endif(RTNET_ROOT_DIR)

set(RTNET_INCLUDE_DIRS ${RTNET_INCLUDE_DIR})
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RTnet DEFAULT_MSG RTNET_INCLUDE_DIRS)
MARK_AS_ADVANCED(RTNET_INCLUDE_DIRS)
