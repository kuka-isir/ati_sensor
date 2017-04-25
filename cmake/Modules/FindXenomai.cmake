################################################################################
#
# CMake script for finding the XENOMAI 2 native or XENOMAI 3 alchemy skin.
# If the optional XENOMAI_ROOT_DIR environment variable exists, header files and
# libraries will be searched in the XENOMAI_ROOT_DIR/include and XENOMAI_ROOT_DIR/lib
# directories, respectively. Otherwise the default CMake search process will be
# used.
#
# This script creates the following variables:
#  XENOMAI_FOUND: Boolean that indicates if the package was found
#  XENOMAI_INCLUDE_DIRS: Paths to the necessary header files
#  XENOMAI_LIBRARIES: Package libraries
#
################################################################################

set(XENOMAI_FOUND TRUE)

# Get hint from environment variable (if any)
if(NOT $ENV{XENOMAI_ROOT_DIR} STREQUAL "")
    set(XENOMAI_ROOT_DIR $ENV{XENOMAI_ROOT_DIR} CACHE PATH "Xenomai base directory location (optional, used for nonstandard installation paths)" FORCE)
    mark_as_advanced(XENOMAI_ROOT_DIR)
endif()

# Find headers and libraries
if(XENOMAI_ROOT_DIR)
    # Use location specified by environment variable
    find_program(XENOMAI_XENO_CONFIG NAMES xeno-config  PATHS ${XENOMAI_ROOT_DIR}/bin NO_DEFAULT_PATH)
else()
    # Use default CMake search process
    find_program(XENOMAI_XENO_CONFIG NAMES xeno-config )
endif()

if(NOT XENOMAI_XENO_CONFIG )
    set(XENOMAI_FOUND FALSE)
    #message(SEND_ERROR "Your Xenomai installation is broken: I can not determine Xenomai cflags/ldflags without xeno-config.")
else()
    execute_process(COMMAND ${XENOMAI_XENO_CONFIG} --version OUTPUT_VARIABLE XENOMAI_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE "." ";" XENOMAI_VERSION_LIST ${XENOMAI_VERSION} )
    list(GET XENOMAI_VERSION_LIST 0 XENOMAI_VERSION_MAJOR)
    list(GET XENOMAI_VERSION_LIST 1 XENOMAI_VERSION_MINOR)
    list(GET XENOMAI_VERSION_LIST 2 XENOMAI_VERSION_PATCH)

    # Here we have xeno-config
    if(${XENOMAI_VERSION_MAJOR} EQUAL 2)
        set(XENOMAI_SKIN_NAME   native)
    endif()

    if(${XENOMAI_VERSION_MAJOR} EQUAL 3)
        set(XENOMAI_SKIN_NAME   alchemy)
        # NOTE: Copperplate auto-initialization
        set(XENOMAI_LDFLAGS_EXTRA_ARGS "--auto-init-solib")
    endif()

    if(NOT XENOMAI_SKIN_NAME)
        message(SEND_ERROR "The only supported Xenomai versions are 2.x and 3.x, your version is ${XENOMAI_VERSION}")
    endif()

    execute_process(COMMAND ${XENOMAI_XENO_CONFIG} --skin=${XENOMAI_SKIN_NAME} --ldflags ${XENOMAI_LDFLAGS_EXTRA_ARGS} OUTPUT_VARIABLE XENOMAI_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${XENOMAI_XENO_CONFIG} --skin=${XENOMAI_SKIN_NAME} --cflags OUTPUT_VARIABLE XENOMAI_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

endif()


if(XENOMAI_LDFLAGS)
    string(STRIP ${XENOMAI_LDFLAGS} XENOMAI_LIBRARY)
    string(REGEX MATCHALL "-I([^ ]+)" XENOMAI_INCLUDE_DIR ${XENOMAI_CFLAGS})
    string(REGEX MATCHALL "-D([^ ]+)" XENOMAI_COMPILE_DEFINITION ${XENOMAI_CFLAGS})
    add_definitions(${XENOMAI_COMPILE_DEFINITION})
    string(REPLACE "-I" ";" XENOMAI_INCLUDE_DIR ${XENOMAI_INCLUDE_DIR})
endif()

find_package_handle_standard_args(Xenomai DEFAULT_MSG  XENOMAI_LIBRARY XENOMAI_INCLUDE_DIR)

set(XENOMAI_LIBRARIES ${XENOMAI_LIBRARY})
set(XENOMAI_INCLUDE_DIRS ${XENOMAI_INCLUDE_DIR})

mark_as_advanced(XENOMAI_LIBRARIES XENOMAI_INCLUDE_DIRS)
