cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

project(tests_seahowl)

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht
             OPTIONAL_COMPONENTS PardisoMKL
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

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
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()


add_executable(${PROJECT_NAME} unit_tests/test_components.cpp ../src/elasto/blade_elasto.cpp ../src/elasto/rotor.cpp ../src/elasto/tower.cpp ../src/io/read_json.cpp ../src/utils.cpp ../src/core/blade_core.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
    LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

target_link_libraries(${PROJECT_NAME} ${CHRONO_LIBRARIES} -lgtest)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_DLL_copy_command("${CHRONO_DLLS}")
