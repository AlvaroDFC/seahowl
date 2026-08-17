# SEAHOWL ↔ OpenFAST Interface

SEAHOWL connects to OpenFAST modules through thin **C-binding adapters**.
Each adapter wraps a Fortran shared library behind a pImpl struct that exposes
`extern "C"` function declarations, so SEAHOWL's fluid and environment layers
never hold a direct Fortran dependency.
Five modules are currently integrated: **AeroDyn** (bundled with **InflowWind**
via the ADI C-bind), **InflowWind** (standalone), **HydroDyn**, **SeaState**,
and **DISCON**.
MoorDyn is not integrated; mooring hydrodynamics use SEAHOWL's internal
Morison model or Chrono FEA cables instead.

---

## Q1 — Where are OpenFAST modules constructed?

| Module | Constructed in | Called by |
|---|---|---|
| `InflowWindAdapter` | `get_environmental_model_from_db()` in `src/io/read_input.cpp` | `IfW_C_Init()` fires inside the constructor |
| `SeaStateAdapter` | `get_environmental_model_from_db()` in `src/io/read_input.cpp` | `SeaSt_C_Init()` fires inside the constructor |
| `TurbineAeroDyn` (owns `AeroDynAdapter`) | `get_turbine_aero_from_db()` in `src/io/read_input.cpp` | `ADI_C_*` called later at `initialize()` |
| `FloaterHydroDyn` / `MonopileHydroDyn` (own `HydroDynAdapter`) | `get_foundation_fluid_from_db()` in `src/io/read_input.cpp` | `HydroDyn_C_Init` called later at `initialize()` |
| `ControllerDISCON` | `get_controller_discon_from_db()` in `src/io/read_input.cpp` | `DisconInterface::Init` (dlopen) called later at `initialize()` |

InflowWind and SeaState are **constructed immediately** when the environment
JSON block is parsed — their Fortran `Init` routines run before the turbine
is even assembled.
The three remaining adapters are constructed at parse time but their Fortran
`Init` routines are deferred to `System::initialize_this()`.

---

## Q2 — What input files / configs select them?

Selection is driven entirely by the JSON input files read in `read_input.cpp`.

| OpenFAST module | JSON key that enables it | File path key |
|---|---|---|
| AeroDyn + InflowWind | `aero.solver = "aerodyn"` inside turbine JSON | `aero.options.file_aerodyn` (AD input file) |
| InflowWind (standalone) | `wind.type = "inflowwind"` inside environment JSON | `wind.options.file` (IfW input file) |
| HydroDyn (floater) | `floater.type = "hydrodyn"` inside floater JSON | `floater.options_file` (HD input file) |
| HydroDyn (monopile) | `foundation.options.solver_hydro = "hydrodyn"` inside turbine JSON | `foundation.options.file_hydrodyn` (HD input file) |
| SeaState | `sea.type = "seastate"` inside environment JSON | `sea.options.file_seastate` (SeaState input file) |
| DISCON | `controller.type = "discon"` inside turbine JSON | `controller.options.libfile` (shared library) · `controller.options.infile` (DISCON parameter file) |

When AeroDyn is active, `TurbineAeroDyn::setup_environment()` automatically
resolves the InflowWind input file from whichever `InflowWindAdapter` is already
registered in `env_model.fluid_models`.
Similarly, `HydroDynAdapter::setup_environment()` resolves the SeaState input
file from the registered `SeaStateAdapter`.
This means the wave / wind file paths are declared once in the environment JSON
and shared across modules automatically.

---

## Q3 — What data does SEAHOWL send into each module?

### AeroDyn + InflowWind (ADI)

Sent via `ADI_C_SetRotorMotion` once per timestep, populated in
`AeroDynAdapter::update_hub/nacelle/roots/mesh_motion()`:

| Quantity | Source in SEAHOWL | Array |
|---|---|---|
| Hub position (m) | `body_hub.get_position()` | `HubPos[3]` |
| Hub orientation (DCM) | `body_hub.get_rotation().toRotationMatrix()` | `HubOri[9]` |
| Hub translational velocity (m/s) | `body_hub.get_velocity()` | `HubVel[3]` |
| Hub rotational velocity (rad/s) | `body_hub.get_rotational_velocity(global)` | `HubVel[3:6]` |
| Hub translational acceleration (m/s²) | `body_hub.get_acceleration()` | `HubAcc[3]` |
| Hub rotational acceleration (rad/s²) | `body_hub.get_rotational_acceleration(global)` | `HubAcc[3:6]` |
| Nacelle pos / ori / vel / acc | same pattern as hub, from `body_nacelle` | `NacPos`, `NacOri`, `NacVel`, `NacAcc` |
| Blade-root pos / ori / vel / acc (×N blades) | from `blade->body_root` | `BldRootPos`, `BldRootOri`, `BldRootVel`, `BldRootAcc` |
| Mesh-node pos / ori / vel / acc (×N×M nodes) | from `blade->nodes[j]` | `MeshPos`, `MeshOri`, `MeshVel`, `MeshAcc` |

### InflowWind (standalone)

Sent via `IfW_C_CalcOutput` on every wind-velocity query:

| Quantity | Source | Array |
|---|---|---|
| Query position (m) | caller position (tower node, blade node, etc.) | `Pos_C[3]` |
| Query time (s) | simulation time | scalar `Time_C` |

### HydroDyn

Sent via `HydroDyn_C_UpdateStates` and implicitly to `CalcOutput`, populated in
`HydroDynAdapter::update_nodes_motion()`:

| Quantity | Source | Array |
|---|---|---|
| Node position x, y, z (m) | `node.get_position()` | `NodePos[i*6:i*6+3]` |
| Node orientation roll, pitch, yaw (rad) | `node.get_rpy_angles()` | `NodePos[i*6+3:i*6+6]` |
| Node translational velocity (m/s) | `node.get_velocity()` | `NodeVel[i*6:i*6+3]` |
| Node rotational velocity (rad/s) | `node.get_rotational_velocity(global)` | `NodeVel[i*6+3:i*6+6]` |
| Node translational acceleration (m/s²) | `node.get_acceleration()` | `NodeAcc[i*6:i*6+3]` |
| Node rotational acceleration (rad/s²) | `node.get_rotational_acceleration(global)` | `NodeAcc[i*6+3:i*6+6]` |

### SeaState

Sent via `SeaSt_C_GetFluidVelAcc` / `SeaSt_C_GetSurfElev` on every query:

| Quantity | Source | Array |
|---|---|---|
| Query position x, y, z (m) | caller position (Morison node, monopile node) | `Pos_c[3]` |
| Query time (s) | simulation time | scalar `Time_C` |

### DISCON

Sent via the `avrSWAP` float array before each `DisconInterface::Call()`:

| Quantity | avrSWAP index | Source in SEAHOWL |
|---|---|---|
| Time (s) | 2 | simulation time |
| Time step (s) | 3 | dt |
| Rotor speed (rad/s) | set via `SetRotorSpeed` | `rna.elasto.get_rpm() × 2π/60` |
| Rotor azimuth (rad) | set via `SetRotorAzimuth` | `rna.elasto.get_azimuth()` |
| Generator speed (rad/s) | set via `SetGeneratorSpeed` | `get_generator_rpm() × 2π/60` |
| Blade pitch per blade (rad) | set via `SetPitchBlade` | `blade.get_pitch()` |
| Blade root moments flapwise/edgewise (Nm) | set via `SetRootMomentBlade` | `blade.get_blade_root_moment()` |
| Tower-top translational accel fore-aft/side-side (m/s²) | set via `SetTowerTopAcceleration` | nacelle accel in local frame |
| Nacelle rotational accel (rad/s²) | set via `SetNacelleRotationalAcceleration` | nacelle rotational accel in shaft frame |
| Yaw error (rad) | set via `SetYawError` | `turbine.rna.get_yaw_error()` |
| Generated power (W) | set via `SetGeneratedPower` | `turbine.get_generated_power()` |
| Shaft power (W) | set via `SetShaftPower` | `turbine.get_shaft_power()` |

---

## Q4 — What data comes back?

### AeroDyn + InflowWind

Returned via `ADI_C_GetRotorLoads` and `ADI_C_GetDiskAvgVel`:

| Quantity | Array / variable | Stored in |
|---|---|---|
| Distributed blade forces + moments at each mesh node | `MeshFrc[6 × NumMeshPts]` | `AeroDynAdapter::forces_aerodyn`, `moments_aerodyn` |
| Disk-averaged wind velocity (m/s) | `DiskAvgVel[3]` | `AeroDynAdapter::disk_averaged_velocity` |

### InflowWind (standalone)

| Quantity | Array | Stored in |
|---|---|---|
| Wind velocity at query point (m/s) | `Vel_C[3]` | returned from `get_velocity_this()` |

### HydroDyn

Returned via `HydroDyn_C_CalcOutput_and_AddedMass`:

| Quantity | Array | Stored in |
|---|---|---|
| Node loads force + moment (N, Nm) | `NodeFrc[6 × NumNodePts]` | `HydroDynAdapter::forces_hydrodyn`, `moments_hydrodyn` |
| Added-mass matrix | `NodeAdm[6N × 6N]` | `HydroDynAdapter::added_mass_matrix` |

### SeaState

| Quantity | Return | Used by |
|---|---|---|
| Fluid velocity (m/s) | `Vel_c[3]` | `get_velocity_this()` → Morison / monopile load calc |
| Fluid acceleration (m/s²) | `Acc_c[3]` | `get_acceleration_this()` → Morison inertia term |
| Surface elevation (m) | `Elev_C` scalar | `get_water_level()` → wave kinematics cut-off |
| Water density (kg/m³) | `Density` scalar | `get_density_this()` → Morison / buoyancy |

### DISCON

Read from `avrSWAP` after `DisconInterface::Call()`:

| Quantity | avrSWAP index | Accessor |
|---|---|---|
| Collective pitch demand (rad) | 45 | `get_collective_pitch()` |
| Blade 1 pitch demand (rad) | 42 | `get_pitch_blade(0)` |
| Blade 2 pitch demand (rad) | 43 | `get_pitch_blade(1)` |
| Blade 3 pitch demand (rad) | 44 | `get_pitch_blade(2)` |
| Generator electrical torque demand (Nm) | 47 | `get_torque_elec()` |
| Yaw rate demand (rad/s) | 48 | `get_yaw_rate()` |

---

## Q5 — Where are returned forces/states applied?

### AeroDyn loads → blade FEA nodes

In `TurbineAeroDyn::compute_env_loads()` (`src/fluid/aero/aerodyn_adapter.cpp`):

```cpp
for (auto& blade : rotor.blades) {
    for (auto& node : blade->nodes) {
        node.load   = aerodyn.forces_aerodyn[count_node];
        node.moment = aerodyn.moments_aerodyn[count_node];
        count_node++;
    }
}
rna->rotor->disk_averaged_wind_velocity = aerodyn.disk_averaged_velocity;
```

The `aero_node.load` / `.moment` fields are then mapped from the aerodynamic
discretization onto the structural (FEA) discretization by `BladeAero` →
`BladeElasto` mesh interpolation during `Turbine::prestep`.

### HydroDyn loads → floater body

In `FloaterHydroDyn::compute_env_loads()` (`src/fluid/hydro/hydrodyn_adapter.cpp`):

```cpp
force_hydro       = hydrodyn->forces_hydrodyn[0];
torque_hydro      = hydrodyn->moments_hydrodyn[0];
added_mass_matrix = rot66.inverse() * hydrodyn->added_mass_matrix;  // rotated to local frame
```

These are consumed by the `FloaterElasto` body inside Chrono at the next
`System::step()`.

### HydroDyn loads → monopile distributed nodes

In `MonopileHydroDyn::compute_env_loads()`:

```cpp
node.load             = hydrodyn->forces_hydrodyn[ii];
node.added_mass_matrix = rot66.inverse() * ...;
```

Applied to each `MorisonNode` along the distributed monopile beam.

### DISCON setpoints → Chrono actuator constraints

The pitch and torque demands retrieved in `ControllerDISCON::step()` are applied
via actuator objects (`ActuatorPitch`, `ActuatorYaw`, generator torque) inside
`Turbine::apply_control()`, which imposes rheonomic constraints on the Chrono
revolute joints before the next `System::step()`.

---

## Q6 — Are modules called during prestep, step, or poststep?

| Phase | Module called | Call |
|---|---|---|
| **Construction** (`read_input`) | InflowWind | `IfW_C_Init()` — inside `InflowWindAdapter` constructor |
| **Construction** (`read_input`) | SeaState | `SeaSt_C_Init()` — inside `SeaStateAdapter` constructor |
| **`initialize_this`** | AeroDyn + InflowWind | `ADI_C_PreInit` · `ADI_C_SetupRotor` · `ADI_C_Init` |
| **`initialize_this`** | HydroDyn | `HydroDyn_C_Init` |
| **`initialize_this`** | DISCON | `DisconInterface::Init` (dlopen) |
| **`prestep` — control** | DISCON | `DisconInterface::Call()` — **first** in prestep, before any fluid call |
| **`prestep` — fluid** | AeroDyn + InflowWind | `ADI_C_SetRotorMotion` · `ADI_C_UpdateStates` · `ADI_C_CalcOutput` · `ADI_C_GetRotorLoads` · `ADI_C_GetDiskAvgVel` |
| **`prestep` — fluid** | HydroDyn | `HydroDyn_C_UpdateStates` · `HydroDyn_C_CalcOutput_and_AddedMass` |
| **`prestep` — on-demand** | InflowWind | `IfW_C_CalcOutput()` — queried per node by BEMT / tower-drag |
| **`prestep` — on-demand** | SeaState | `SeaSt_C_GetFluidVelAcc` / `SeaSt_C_GetSurfElev` — queried per node by Morison |
| **`step`** | *(none)* | Pure Chrono structural integration |
| **`poststep`** | *(none)* | Elasto→fluid position sync only |

The call order within `prestep` is:
**[1] DISCON** → **[2] AeroDyn + HydroDyn** (parallel, both inside `apply_env_model`) →
**[3] on-demand InflowWind / SeaState** (triggered inside BEMT / Morison loops).

---

## Diagram

```mermaid
{% include "./diagrams/openfast_interface_map.mmd" %}
```

---

## Unresolved Issues

### U1 — MoorDyn not integrated

MoorDyn is absent from the codebase. No `moordyn_adapter.*` file exists.
Mooring loads are provided either by SEAHOWL's internal Morison elements
(`MooringHydro` / `MooringSystemHydro`) or by Chrono FEA cables
(`MooringElastoFEA`).
**Open question:** Is MoorDyn planned as a future adapter or deliberately excluded?

### U2 — AeroDyn external flow-field sharing disabled (FIXME present)

`AeroDynInflowLib::Init()` contains a commented-out block:

```cpp
// FIXME: if using external flow field, set it here
// if (externFlowField == 1) { SetFlowFieldPointer(FlowFieldPtr); }
```

`ADI_C_GetFlowFieldPointer`, `ADI_C_SetFlowFieldPointer`, `IfW_C_GetFlowFieldPointer`,
and `IfW_C_SetFlowFieldPointer` are all declared but never called.
The `externFlowField` flag is hardcoded to `0`.
**Open question:** Sharing the wave/wind field pointer between a standalone InflowWind
instance and AeroDyn's internal InflowWind is not yet wired up. Planned?

### U3 — SeaState `PreInit` call status unclear

`SeaSt_C_PreInit()` is declared and its signature is fully matched in
`seastate_adapter.cpp`, but no call to it was found in `SeaStateLib::Init()`.
**Open question:** Is `PreInit` needed before `SeaSt_C_Init`, or has the SeaState
C-binding API changed to fold PreInit into Init?

### U4 — Force-to-elasto transfer not fully traced

This document traces forces to `aero_node.load`, `force_hydro`, and
`added_mass_matrix`. The subsequent path through
`Turbine::update_loads_elasto()` → Chrono generalized forces was not inspected.
**Open question:** Confirm in `src/core/component_elasto_fluid.cpp` that all
three load quantities reach the Chrono body/FEA system correctly.

### U5 — DISCON yaw control hardcoded off

```cpp
pImpl.SetAvrSWAP(29, 0.0);  // yaw rate control
```

This is unconditional in `ControllerDISCON::initialize()`.
**Open question:** Is there a turbine type or config flag that is expected to
enable yaw rate control via DISCON?

### U6 — Tower shadow double-counting risk with AeroDyn

`TurbineAero::setup_environment()` still populates `TowerAero` nodes when
AeroDyn is active. `RotorAeroDyn::compute_env_loads()` is a deliberate no-op
(tower shadow is handled inside AeroDyn's Fortran), but the tower drag
(`TowerAero::compute_env_loads`) still runs on the SEAHOWL side.
**Open question:** Verify that AeroDyn's tower-shadow model and SEAHOWL's
independent tower-drag computation do not produce double-counted loads on the
tower structure.

### U7 — HydroDyn added-mass variant always used

`HydroDynLib::Calcul()` calls only `HydroDyn_C_CalcOutput_and_AddedMass`.
The original `HydroDyn_C_CalcOutput` (which also accepts node accelerations)
is commented out.
**Open question:** Is the added-mass matrix always required (e.g. for correct
floating platform dynamics), or should the non-added-mass variant be available
as a fallback for performance-sensitive monopile cases?
