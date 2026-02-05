# C++ API {#mainpage}

SEAHOWL is a time-domain multi-physics simulation framework written in C++ and primarily developed for wind turbine analysis.

The C++ library is organized into namespaces, each covering a specific physical domain or functional area of the wind turbine simulation pipeline, as described in the table below.

| Namespace | Description |
|-----------|-------------|
| seahowl | Top-level utilities for entity management, logging, and component interactions. |
| seahowl::core | Multi-body turbine system orchestration: turbines, towers, rotors, blades, foundations, and moorings. |
| seahowl::elasto | Structural dynamics and elastodynamics using finite element and multibody methods (Chrono). |
| seahowl::fluid::aero | Aerodynamic load computation such as Blade Element Momentum Theory (BEMT). |
| seahowl::fluid::hydro | Hydrodynamic forces on substructures using Morison equations and potential flow. |
| seahowl::servo | Turbine control systems: pitch and torque controllers, DISCON integration. |
| seahowl::env | Environmental conditions: wind fields, wave models, and soil models. |
| seahowl::io | Input/output: configuration parsing, result export, and visualization. |
