cmake_minimum_required(VERSION 3.10)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# ==================
# SEAHOWL PROJECT
# =================
project( SEAHOWL 
         VERSION 0.0.1
         DESCRIPTION "Wind Turbine simulation with CHRONO"
         LANGUAGES CXX
)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()


LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")  # TO BE CLEANDED

# Add the cmake folder so the FindSphinx module is found
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# =============
# OPTIONS
# =============

option (SEAHOWL_ENABLE_IRRLICHT "Enable Irrlicht library" ON)
option (SEAHOWL_ENABLE_TESTS "Enable tests" ON)
option (SEAHOWL_ENABLE_DOC "Generate html documentation" ON)
option (SEAHOWL_ENABLE_BUILD "Build library and drivers" ON)
option (SEAHOWL_ENABLE_PYTHON "Enable python binding" ON)

# =======================
# External libraries
# =======================
if(SEAHOWL_ENABLE_IRRLICHT)
    set(IRRLICHT_COMP Irrlicht )
    add_definitions("-DHAVE_IRRLICHT=1")
endif(SEAHOWL_ENABLE_IRRLICHT)

find_package(Chrono
            COMPONENTS ${IRRLICHT_COMP}
            OPTIONAL_COMPONENTS PardisoMKL
            CONFIG REQUIRED)



find_package(nlohmann_json CONFIG REQUIRED)



if( NOT CMAKE_SIZEOF_VOID_P EQUAL 8 )
	message(FATAL_ERROR "ONLY 64 BITS SUPPORTED")
endif()

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(/opt/homebrew/include)  ## TO BE REMOVED


if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface

    ## TO BE REMOVED: SHOULD BE ALREADY SET BY CHRONO LIBRARY
    if(${MSVC_VERSION} GREATER_EQUAL 1915)         # Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
        add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
    endif()
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()

# ==============
# DATADIR
# ==============
# Define a data directory to find test input files
#
# If the directory is not set as environment variable 
# set a default value in CMake variable. This default
# value can be modified in the CMakeCache.txt file
# in build directory.
#
if(DEFINED ENV{SEAHOWL_DATADIR})
	set(SEAHOWL_DATADIR $ENV{SEAHOWL_DATADIR})
else()
	set(SEAHOWL_DATADIR "${PROJECT_SOURCE_DIR}/data")
endif()


# ================
# CORE LIBRARY
# ================
add_library( seahowl_core )



set(SEAHOWL_SOURCES 

	src/elasto/blade_elasto.cpp 
    src/elasto/rotor_elasto.cpp
    src/elasto/tower_elasto.cpp
    
    src/core/rotor.cpp 
    src/core/blade.cpp
    src/core/turbine.cpp
    src/core/utils.cpp 

    src/aero/blade_aero.cpp 
    src/aero/rotor_aero.cpp
    src/servo/controller.cpp

	src/io/read_json.cpp 	 
)

set(SEAHOWL_HEADERS

    include/seahowl/servo/controller.h
)

target_sources(seahowl_core 
    PRIVATE 
    ${SEAHOWL_SOURCES}
    ${SEAHOWL_HEADERS}
)

target_include_directories(seahowl_core

	PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>

	PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CHRONO_INCLUDE_DIRS}
)

target_compile_features(
	seahowl_core

	PRIVATE
	cxx_std_17
)

set_target_properties(seahowl_core

    PROPERTIES
        CXX_EXTENSIONS OFF
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)


target_link_libraries(seahowl_core
    PRIVATE 
    nlohmann_json::nlohmann_json
    ${CHRONO_LIBRARIES}
)

# =============
# DRIVER 
# =============
add_executable( seahowl_driver )


target_sources(seahowl_driver 
    PRIVATE 
    src/main.cpp

    ${SEAHOWL_HEADERS}
)

target_include_directories(seahowl_driver

	PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>

    PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CHRONO_INCLUDE_DIRS}
)

target_compile_features(
	seahowl_driver

	PRIVATE
	cxx_std_17
)

set_target_properties(seahowl_driver 

    PROPERTIES
        CXX_EXTENSIONS OFF
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)


target_link_libraries(seahowl_driver 
    PRIVATE 
    seahowl_core
    nlohmann_json::nlohmann_json
    ${CHRONO_LIBRARIES}
)

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# Note that you must first set EXECUTABLE_OUTPUT_PATH
# (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
#--------------------------------------------------------------

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_DLL_copy_command("${CHRONO_DLLS}")

# ===========
# TESTS
# ===========
if(SEAHOWL_ENABLE_TESTS)

	include(CTest)
	enable_testing()

	add_subdirectory(tests)

endif(SEAHOWL_ENABLE_TESTS)

# ====================
# DOCUMENTATION
# ====================
if(SEAHOWL_ENABLE_DOC)
	add_subdirectory(doc)
endif(SEAHOWL_ENABLE_DOC)