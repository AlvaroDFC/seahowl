include(vcpkg_find_fortran)

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/openfast/openfast.git
    REF 02847314dfdc4069d70b8b493e37b12b810a11e2
    PATCHES
        "openfast_custom_command.patch"
)

vcpkg_find_fortran(FORTRAN_CMAKE)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FORTRAN_CMAKE}
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake
                         PACKAGE_NAME OpenFAST
                         NO_PREFIX_CORRECTION)
