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
include(LibFindMacros)

# Get hint from environment variable (if any)
if(NOT $ENV{XENOMAI_ROOT_DIR} STREQUAL "")
    set(XENOMAI_ROOT_DIR $ENV{XENOMAI_ROOT_DIR} CACHE PATH "Xenomai base directory location (optional, used for nonstandard installation paths)" FORCE)
    mark_as_advanced(XENOMAI_ROOT_DIR)
endif()

if ( Xenomai_FIND_QUIETLY )
    set( XENOMAI_FIND_QUIETLY "QUIET")
endif()

if ( Xenomai_FIND_REQUIRED )
    set( XENOMAI_FIND_REQUIRED "REQUIRED")
endif()

# Find headers and libraries
if(XENOMAI_ROOT_DIR)
    # Use location specified by environment variable
    find_program(XENOMAI_XENO_CONFIG NAMES xeno-config  PATHS ${XENOMAI_ROOT_DIR}/bin NO_DEFAULT_PATH)
else()
    # Use default CMake search process
    find_program(XENOMAI_XENO_CONFIG NAMES xeno-config )
endif()

if(XENOMAI_XENO_CONFIG )
    # Detect Xenomai version
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
        # NOTE: --auto-init-solib bootstraps xenomai_init()
        set(XENO_CONFIG_LDFLAGS_EXTRA_ARGS "--auto-init-solib")
    endif()

    if(NOT XENOMAI_SKIN_NAME)
        message(FATAL_ERROR "Only Xenomai 2.x and 3.x are supported, your version is ${XENOMAI_VERSION}")
    endif()

    message(STATUS "Xenomai ${XENOMAI_VERSION} detected, searching for ${XENOMAI_SKIN_NAME} skin.")

    execute_process(COMMAND ${XENOMAI_XENO_CONFIG} --skin=${XENOMAI_SKIN_NAME} --ldflags ${XENO_CONFIG_LDFLAGS_EXTRA_ARGS}
                    OUTPUT_VARIABLE XENOMAI_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_VARIABLE XENOMAI_LDFLAGS_ERROR)
    execute_process(COMMAND ${XENOMAI_XENO_CONFIG} --skin=${XENOMAI_SKIN_NAME} --cflags ${XENOMAI_COMPAT}
                    OUTPUT_VARIABLE XENOMAI_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_VARIABLE XENOMAI_CFLAGS_ERROR)

    if( XENOMAI_LDFLAGS_ERROR)
        message(FATAL_ERROR "Could not determine ldflags with command ${XENOMAI_XENO_CONFIG} --skin=${XENOMAI_SKIN_NAME} --ldflags ${XENO_CONFIG_LDFLAGS_EXTRA_ARGS}")
    endif()

    if( XENOMAI_CFLAGS_ERROR)
        message(FATAL_ERROR "Could not determine cflags with command ${XENOMAI_XENO_CONFIG} --skin=${XENOMAI_SKIN_NAME} --cflags ${XENO_CONFIG_LDFLAGS_EXTRA_ARGS}")
    endif()

    string(REGEX MATCHALL "-L([^ ]+)|-l([^ ]+)" XENOMAI_LIBRARY ${XENOMAI_LDFLAGS})
    string(REGEX MATCHALL "-I([^ ]+)" XENOMAI_INCLUDE_DIR ${XENOMAI_CFLAGS})
    string(REGEX MATCHALL "-D([^ ]+)" XENOMAI_COMPILE_DEFINITIONS ${XENOMAI_CFLAGS})
    string(REPLACE "-I" ";" XENOMAI_INCLUDE_DIR ${XENOMAI_INCLUDE_DIR})

    message(STATUS "
    ==========================================
    Xenomai ${XENOMAI_VERSION} ${XENOMAI_SKIN_NAME} skin
        libs    : ${XENOMAI_LIBRARY}
        include : ${XENOMAI_INCLUDE_DIR}
        ldflags : ${XENOMAI_LDFLAGS}
        cflags  : ${XENOMAI_CFLAGS}
    ==========================================
    ")

endif()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(XENOMAI_PROCESS_INCLUDES XENOMAI_INCLUDE_DIR)
set(XENOMAI_PROCESS_LIBS XENOMAI_LIBRARY)

libfind_process(XENOMAI)
