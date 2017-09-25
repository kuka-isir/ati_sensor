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

# Get hint from environment variable (if any)
if(NOT $ENV{XENOMAI_ROOT_DIR} STREQUAL "")
    set(XENOMAI_ROOT_DIR $ENV{XENOMAI_ROOT_DIR} CACHE PATH "Xenomai base directory location (optional, used for nonstandard installation paths)" FORCE)
    mark_as_advanced(XENOMAI_ROOT_DIR)
endif()

if (Xenomai_FIND_QUIETLY)
    set(XENOMAI_FIND_QUIETLY "QUIET")
endif()

if (Xenomai_FIND_REQUIRED)
    set(XENOMAI_FIND_REQUIRED "REQUIRED")
endif()

# Find headers and libraries
if(XENOMAI_ROOT_DIR)
    # Use location specified by environment variable
    find_program(XENOMAI_XENO_CONFIG NAMES xeno-config  PATHS ${XENOMAI_ROOT_DIR}/bin NO_DEFAULT_PATH)
else()
    # Use default CMake search process
    find_program(XENOMAI_XENO_CONFIG NAMES xeno-config )
endif()

function(find_xeno_skin_variables prefix skin_name)
    message(STATUS "Xenomai ${${prefix}_VERSION} detected, searching for ${skin_name} skin.")

    execute_process(COMMAND ${XENOMAI_XENO_CONFIG} --skin=${skin_name} --ldflags ${XENO_CONFIG_LDFLAGS_EXTRA_ARGS}
                    OUTPUT_VARIABLE ${prefix}_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_VARIABLE ${prefix}_LDFLAGS_ERROR)
    execute_process(COMMAND ${XENOMAI_XENO_CONFIG} --skin=${skin_name} --cflags
                    OUTPUT_VARIABLE ${prefix}_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_VARIABLE ${prefix}_CFLAGS_ERROR)

    if(${prefix}_LDFLAGS_ERROR)
        message(FATAL_ERROR "Could not determine ldflags with command ${XENOMAI_XENO_CONFIG} --skin=${skin_name} --ldflags ${XENO_CONFIG_LDFLAGS_EXTRA_ARGS}")
    endif()

    if(${prefix}_CFLAGS_ERROR)
        message(FATAL_ERROR "Could not determine cflags with command ${XENOMAI_XENO_CONFIG} --skin=${skin_name} --cflags")
    endif()

    set(${prefix}_FOUND TRUE)

    if(NOT ${${prefix}_LDFLAGS} STREQUAL "")
        string(REGEX MATCHALL "-L([^ ]+)|-l([^ ]+)" ${prefix}_LIBRARY ${${prefix}_LDFLAGS})
    else()
        set(${prefix}_LIBRARY "")
    endif()
    string(REGEX MATCHALL "-I([^ ]+)" ${prefix}_INCLUDE_DIR ${${prefix}_CFLAGS})
    string(REGEX MATCHALL "-D([^ ]+)" ${prefix}_COMPILE_DEFINITIONS ${${prefix}_CFLAGS})
    string(REPLACE "-I" ";" ${prefix}_INCLUDE_DIR ${${prefix}_INCLUDE_DIR})

    message(STATUS "
--------------------------------------------------------------------------------
    Xenomai ${XENOMAI_VERSION} ${skin_name} skin
        libs    : ${${prefix}_LIBRARY}
        include : ${${prefix}_INCLUDE_DIR}
        ldflags : ${${prefix}_LDFLAGS}
        cflags  : ${${prefix}_CFLAGS}")

        set(${prefix}_INCLUDE_DIRS ${${prefix}_INCLUDE_DIR} CACHE INTERNAL "")
        set(${prefix}_LIBRARIES ${${prefix}_LIBRARY} CACHE INTERNAL "")
        set(${prefix}_COMPILE_DEFINITIONS ${${prefix}_COMPILE_DEFINITIONS} CACHE INTERNAL "")
        set(${prefix}_LDFLAGS ${${prefix}_LDFLAGS} CACHE INTERNAL "")
        set(${prefix}_CFLAGS ${${prefix}_CFLAGS} CACHE INTERNAL "")
        set(${prefix}_FOUND ${${prefix}_FOUND} CACHE INTERNAL "")

        mark_as_advanced(${prefix}_LIBRARIES ${prefix}_INCLUDE_DIRS ${prefix}_COMPILE_DEFINITIONS ${prefix}_CFLAGS ${prefix}_LDFLAGS)

    message(STATUS "
--------------------------------------------------------------------------------
    ")
endfunction()

function(handle_standard_args prefix)
    find_package_handle_standard_args(${prefix} DEFAULT_MSG ${prefix}_LIBRARIES ${prefix}_INCLUDE_DIRS ${prefix}_COMPILE_DEFINITIONS ${prefix}_CFLAGS ${prefix}_LDFLAGS)
endfunction()

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

    find_xeno_skin_variables(XENOMAI ${XENOMAI_SKIN_NAME})
    find_xeno_skin_variables(XENOMAI_POSIX posix)
    find_xeno_skin_variables(XENOMAI_RTDM rtdm)
else()
    set(XENOMAI_FOUND FALSE)
    set(XENOMAI_POSIX_FOUND FALSE)
    set(XENOMAI_RTDM_FOUND FALSE)
endif()

handle_standard_args(XENOMAI)
handle_standard_args(XENOMAI_POSIX)
handle_standard_args(XENOMAI_RTDM)
