## Installation


### Dependencies

- Chrono:
  - repo: https://github.com/projectchrono/chrono
  - version: 7.0.3

- json:
  - repo: https://github.com/nlohmann/json
  - version: 3.10.5

- ROSCO (optional):
  - repo: https://github.com/NREL/ROSCO
  - version: 2.5.1

For documentation:

- Doxygen (optional):
  - repo: https://doxygen.nl/
  - version: 1.8.0+

- Graphviz (optional):
  - repo: https://graphviz.org/
  - version: latest

- Sphinx (optional):
  - repo: https://www.sphinx-doc.org/en/master/
  - version: 4.4.0+

For visualization:

  - IRRLICHT (optional):
    - repo: https://irrlicht.sourceforge.io/
    - version: latest

For tests:

  - Google tests:
    - repo: https://github.com/google/googletest
    - version: latest
    
### Compilation

From a directory `./build`:

```bash
cmake .. -DChrono_DIR=/path/to/your/chrono/cmake/build/directory
make
```

#### Default CMAKE Options:

```cmake
option (SEAHOWL_ENABLE_IRRLICHT "Enable Irrlicht 3D visualization library" OFF)
option (SEAHOWL_ENABLE_TESTS "Enable tests" ON)
option (SEAHOWL_ENABLE_DOC "Generate html documentation" ON)
option (SEAHOWL_ENABLE_BUILD "Build library and drivers" ON)
option (SEAHOWL_ENABLE_PYTHON "Enable python binding" ON)
option (SEAHOWL_ENABLE_EXAMPLES "Enable examples" ON)
option (SEAHOWL_ENABLE_ROSCO "Enable ROSCO controller" ON)
option (SEAHOWL_ENABLE_VTK_OUTPUT "Enable VTK library for output" OFF)

```

