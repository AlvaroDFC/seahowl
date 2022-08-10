## Installation


### Dependencies

- Chrono:
  - repo: https://github.com/projectchrono/chrono
  - version: 7.0.3

- json:
  - repo: https://github.com/nlohmann/json
  - version: 3.10.5

- ROSCO:
  - repo: https://github.com/NREL/ROSCO
  - version: 2.5.1

### Compilation

From a directory `./build`:
```
cmake .. -DChrono_DIR=/path/to/your/chrono/cmake/build/directory
make
```

Default CMAKE Options:

```cmake
option (SEAHOWL_ENABLE_IRRLICHT "Enable Irrlicht 3D visualization library" OFF)
option (SEAHOWL_ENABLE_TESTS "Enable tests" ON)
option (SEAHOWL_ENABLE_DOC "Generate html documentation" ON)
option (SEAHOWL_ENABLE_BUILD "Build library and drivers" ON)
option (SEAHOWL_ENABLE_PYTHON "Enable python binding" ON)
option (SEAHOWL_ENABLE_EXAMPLES "Enable examples" ON)
option (SEAHOWL_ENABLE_ROSCO "Enable ROSCO controller" ON)

```

