# ===================
# MKDOCS
# ===================


if(SEAHOWL_ENABLE_PYTHON)
  # Cherche l'exécutable mkdocs dans le système ou l'environnement virtuel
  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_program(MKDOCS_EXECUTABLE NAMES mkdocs PATHS "${CMAKE_BINARY_DIR}/.venv/bin" NO_DEFAULT_PATH)
    if(NOT MKDOCS_EXECUTABLE)
      find_program(MKDOCS_EXECUTABLE NAMES mkdocs)
    endif()
  else()
    find_program(MKDOCS_EXECUTABLE NAMES mkdocs PATHS "${CMAKE_BINARY_DIR}/.venv/Scripts" NO_DEFAULT_PATH)
    if(NOT MKDOCS_EXECUTABLE)
      find_program(MKDOCS_EXECUTABLE NAMES mkdocs)
    endif()
  endif()

  if(NOT MKDOCS_EXECUTABLE)
	  message(WARNING "Can't build SEAHOWL's docs. mkdocs not found!")
    return()
  endif()

  set(MKDOCS_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/source)
  set(MKDOCS_INDEX_FILE ${CMAKE_BINARY_DIR}/doc/html/source/index.html)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
          set(ACTIVATE_PY "${CMAKE_BINARY_DIR}/.venv/bin/activate")
          file(WRITE ${CMAKE_BINARY_DIR}/run_mkdocs.sh
          "#!/bin/bash\n"
          "export PYTHONPATH=${CMAKE_BINARY_DIR}:$PYTHONPATH\n"
          "source ${ACTIVATE_PY}\n"
          "# Generate stub files for pybind11 module (required for mkdocstrings)\n"
          "pybind11-stubgen seahowl -o ${CMAKE_BINARY_DIR} 2>/dev/null || true\n"
          "${MKDOCS_EXECUTABLE} build\n"
          )
          set(FILE_MKDOCS ${CMAKE_BINARY_DIR}/run_mkdocs.sh)
          set(LAUNCH_MKDOCS bash)

  else()
          set(ACTIVATE_PY "${CMAKE_BINARY_DIR}/.venv/Scripts/activate")
          file(WRITE ${CMAKE_BINARY_DIR}/run_mkdocs.bat
          "set PYTHONPATH=${CMAKE_BINARY_DIR};%PYTHONPATH%\n"
          "call ${ACTIVATE_PY}\n"
          "REM Generate stub files for pybind11 module (required for mkdocstrings)\n"
          "pybind11-stubgen seahowl -o ${CMAKE_BINARY_DIR} 2>nul\n"
          "call ${MKDOCS_EXECUTABLE} build\n"
          )
          set(FILE_MKDOCS ${CMAKE_BINARY_DIR}/run_mkdocs.bat)
          set(LAUNCH_MKDOCS cmd.exe /c)
  endif()

  # Make the script executable
  file(COPY ${FILE_MKDOCS} DESTINATION ${CMAKE_BINARY_DIR})
  file(CHMOD ${FILE_MKDOCS} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

  add_custom_command(OUTPUT ${MKDOCS_INDEX_FILE}
      COMMAND ${LAUNCH_MKDOCS} ${FILE_MKDOCS}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      DEPENDS ${MKDOCS_SOURCE} # Every source files
      MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/mkdocs.yml
      COMMENT "Generating documentation with MkDocs"
      VERBATIM
  )

  add_custom_target(mkdocs DEPENDS pyseahowl ${MKDOCS_SOURCE} ${MKDOCS_INDEX_FILE})
endif(SEAHOWL_ENABLE_PYTHON)
# ===================
# DOXYGEN
# ===================
find_package(Doxygen
    REQUIRED dot
    OPTIONAL_COMPONENTS mscgen dia)

    if (DOXYGEN_FOUND)

    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    add_custom_target( doxygen
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating documentation (Doxygen)"
      VERBATIM
    )

  else (DOXYGEN_FOUND)
    message("Doxygen needs to be install. You can use `sudo apt install doxygen` on ubuntu")
  endif (DOXYGEN_FOUND)

if(SEAHOWL_ENABLE_PYTHON)
  set(TARGET_DOC mkdocs doxygen)
else(SEAHOWL_ENABLE_PYTHON)
  set(TARGET_DOC doxygen)
endif(SEAHOWL_ENABLE_PYTHON)

add_custom_target(docs DEPENDS ${TARGET_DOC})

add_custom_command(TARGET docs POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_BINARY_DIR}/doc/html
        ${CMAKE_BINARY_DIR}/doc/source/doxygen
    COMMENT "Copying Doxygen HTML into MkDocs site"
)

# Remove target attributes from Doxygen HTML/SVG so links stay inside the MkDocs iframe
add_custom_command(TARGET docs POST_BUILD
    COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/patch_doxygen_targets.py
        ${CMAKE_BINARY_DIR}/doc/source/doxygen
    COMMENT "Patching Doxygen HTML/SVG for iframe compatibility"
)
if(SEAHOWL_ENABLE_PYTHON)
  add_dependencies(docs pyseahowl)
  # For MSVC IDE
  set_target_properties(
    docs ${TARGET_DOC}
    PROPERTIES
    FOLDER DOC
  )
endif(SEAHOWL_ENABLE_PYTHON)
