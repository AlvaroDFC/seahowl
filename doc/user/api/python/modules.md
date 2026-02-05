# Python API

SEAHOWL exposes its simulation framework through a set of deep Python bindings.
Each module covers a specific physical domain or functional area of the
offshore wind turbine simulation pipeline, as described in the table below.

| Module | Description |
|--------|-------------|
| [seahowl](seahowl.md) | Top-level utilities for entity management, logging, and component interactions. |
| [seahowl.core](core.md) | Multi-body turbine system orchestration: turbines, towers, rotors, blades, foundations, and moorings. |
| [seahowl.elasto](elasto.md) | Structural dynamics and elastodynamics using finite element and multibody methods. |
| [seahowl.aero](aero.md) | Aerodynamic load computation such as Blade Element Momentum Theory (BEMT). |
| [seahowl.hydro](hydro.md) | Hydrodynamic forces on substructures using Morison equations and potential flow. |
| [seahowl.servo](servo.md) | Turbine control systems: pitch and torque controllers, DISCON integration. |
| [seahowl.env](env.md) | Environmental conditions: wind fields, wave models,and soil models. |
| [seahowl.io](io.md) | Input/output: configuration parsing, result export, and visualization. |
