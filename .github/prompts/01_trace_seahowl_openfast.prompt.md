# Study SEAHOWL interface to OpenFAST modules
Goal:
Get up to speed on the existing interface between SEAHOWL and OpenFAST.

Scope:
- AeroDyn adapter
- HydroDyn adapter
- InflowWind adapter
- SeaState adapter
- DISCON controller
- MoorDyn if present

Deliverables:
- doc/architecture/seahowl_openfast_interface.md
- doc/architecture/tables/openfast_module_matrix.md
- doc/architecture/diagrams/openfast_interface_map.mmd

Questions:
1. Where are OpenFAST modules constructed?
2. What input files/configs select them?
3. What data does SEAHOWL send into each module?
4. What data comes back?
5. Where are returned forces/states applied?
6. Are modules called during prestep, step, or poststep?