vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/NREL/HydroChrono.git
    REF 3a191dc328f3ec6d8e3e295a2365f43b7ef2e6a8
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DHYDROCHRONO_ENABLE_IRRLICHT=OFF
        -DHYDROCHRONO_ENABLE_DEMOS=OFF
        -DHYDROCHRONO_ENABLE_TESTS=OFF
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME HydroChrono
                         NO_PREFIX_CORRECTION )
