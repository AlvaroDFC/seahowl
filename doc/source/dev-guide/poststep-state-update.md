# Poststep: State Synchronization & Output

This document details the call chain executed during the `poststep` phase of each simulation step.
It complements the high-level description in [sim-workflow.md](sim-workflow.md) and the prestep
analysis in [prestep-env-loads.md](prestep-env-loads.md).

---

## Position in the Simulation Loop

```
Simulation::step()
│
├─ system_core->prestep(time, dt)       ← loads computed, transferred to elasto
├─ system_core->step(dt)                ← Chrono DoStepDynamics(dt), time advances to t_{n+1}
├─ system_core->poststep(time, dt)      ← THIS DOCUMENT — read back deformed state
│
└─ if (time >= t_output_next):
       outputs->output_all(nstep)       ← writes CSV/VTK using freshly synced aero/hydro positions
```

**Key timing note:** By the time `poststep` executes, Chrono has already advanced the system to `t_{n+1}`.
The elasto nodes hold the new deformed positions. Poststep reads these back into the fluid domain so that
the *next* prestep can compute loads at the correct geometry.

**Data flow direction:** Prestep transfers loads from the fluid domain to the structural domain
(fluid → elasto). Poststep transfers kinematics in the reverse direction (elasto → fluid).

---

## Entry Point: `System::poststep`

```cpp
// system.cpp:153
void System::poststep(double time, double dt) {
    for (auto& turbine : turbines)
        turbine->poststep(time, dt);
    for (auto& component : components)
        component->poststep(time, dt);
}
```

Symmetric structure to `System::apply_env_model` — same iteration over turbines then standalone components.

---

## Turbine Poststep — Full Call Tree

```
Turbine::poststep(time, dt)                         [turbine.cpp]
│
├─ 1. rna.elasto.reset_loads()
│       Zeroes all external forces on the RNA elasto bodies so they don't
│       accumulate across steps.
│
├─ 2. Gearbox efficiency torque correction:
│       aero_torque = rna.elasto.get_axial_torque()
│       if controller.has_torque_control:
│           aero_torque += controller.get_torque_elec() * gearbox_ratio * gearbox_efficiency
│       rna.elasto.rotor->accumulate_axial_torque(-aero_torque * (1 - gearbox_efficiency))
│
│       ↑ Applies a resistive torque representing mechanical losses in the drivetrain.
│         Uses the torque from the *current* step result, which is why it lives in
│         poststep rather than prestep.
│
├─ 3. rna.poststep(time, dt)            → see §RNA / Rotor / Blade
├─ 4. tower.poststep(time, dt)          → see §Tower
├─ 5. foundation->poststep(time, dt)    → see §Foundation / Floater (if foundation exists)
└─ 6. controller->poststep(time, dt, *this)  → see §Controller Poststep
```

---

## RNA / Rotor / Blade: Elasto → Aero Position Sync

```
RotorNacelleAssembly::poststep(time, dt)            [rotor.cpp]
│
├─ rotor.poststep(time, dt)
│    └─ for each blade:
│         blade->poststep(time, dt)                  [blade.cpp]
│              └─ update_positions_aero()
│                   for each aero node [ii]:
│                       read elasto node position/rotation/velocity/acceleration
│                       write into aero.nodes[ii]
│
└─ update_positions_aero()   ← RNA-level: syncs hub & nacelle
        aero.rotor->pitch_collective  = elasto.rotor->pitch_collective
        aero.rotor->azimuth           = elasto.get_azimuth()
        aero.rotor->body_hub          ← copy 6-DOF state from elasto hub
        aero.body_nacelle             ← copy 6-DOF state from elasto nacelle
```

After Chrono integration, blade FEA nodes have new deformed positions. This maps them into the aero
discretization so the next `compute_env_loads` sees correct geometry. The transferred quantities are:
position, rotation, velocity, rotational velocity, acceleration, and rotational acceleration.

---

## Tower: Elasto → Aero Position Sync

```
Tower::poststep(time, dt)                           [tower.cpp]
│
└─ update_positions_aero()
     for each aero node [ii]:
         elasto_element_index = mapping[ii].index
         eta                  = mapping[ii].eta
         entity = elasto.get_entity_along_component(eta, elasto_element_index)
         aero.nodes[ii].set_position(entity.get_position())
         aero.nodes[ii].set_rotation(entity.get_rotation())
         aero.nodes[ii].set_velocity(entity.get_velocity())
         aero.nodes[ii].set_rotational_velocity(entity.get_rotational_velocity())
         aero.nodes[ii].set_acceleration(entity.get_acceleration())
         aero.nodes[ii].set_rotational_acceleration(entity.get_rotational_acceleration())
```

Same interpolation pattern as blade — uses `(element_index, eta)` mapping between the elasto and aero
discretizations which generally have different node counts and spacing.

---

## Foundation / Floater Poststep

```
Floater::poststep(time, dt)                         [floater.cpp]
│
├─ mooring_system->poststep(time, dt)               [mooring.cpp]
│    └─ for each mooring line:
│         mooring->poststep(time, dt)
│              └─ update_positions_hydro()
│                   for each hydro node [ii]:
│                       elasto_element_index = mapping[ii].index
│                       eta = mapping[ii].eta
│                       entity = elasto.get_entity_along_component(eta, elasto_element_index)
│                       node_hydro.set_position(entity.get_position())
│                       node_hydro.set_rotation(entity.get_rotation())
│                       node_hydro.set_velocity(entity.get_velocity())
│                       node_hydro.set_rotational_velocity(entity.get_rotational_velocity())
│                       node_hydro.set_acceleration(entity.get_acceleration())
│                       node_hydro.set_rotational_acceleration(entity.get_rotational_acceleration())
│
└─ update_positions_hydro()   ← floater main body (rigid body, not FEA)
     hydro.body_main.set_position(elasto.body_main.get_position())
     hydro.body_main.set_rotation(elasto.body_main.get_rotation())
     hydro.body_main.set_velocity(elasto.body_main.get_velocity())
     hydro.body_main.set_rotational_velocity(elasto.body_main.get_rotational_velocity())
     hydro.body_main.set_acceleration(elasto.body_main.get_acceleration())
     hydro.body_main.set_rotational_acceleration(elasto.body_main.get_rotational_acceleration())
```

Mooring lines (FEA in elasto) have new catenary shapes after the Chrono step; the floater rigid body has
a new 6-DOF state. All quantities are copied to the hydro domain so the next HydroDyn/mooring load
evaluation operates on up-to-date kinematics.

---

## Controller Poststep

```
Controller::poststep(time, dt, turbine)             [controller.cpp]
    ← base class: NO-OP

ControllerVariableTorque::poststep(...)             [controller.cpp]
    torque_elec_previous = torque_elec
    ← saves current torque for rate-limiting in the next step

ControllerDISCON::poststep(...)
    ← NOT overridden; inherits no-op from base class
```

The actual controller computation (`Controller::step()`) happens during prestep's `apply_control` phase.
Poststep is only used for bookkeeping — storing previous values needed for derivative or rate-limit
calculations at the next step.

---

## Output Phase (immediately after poststep)

```
Simulation::step() [continued after poststep]:       [simulation.cpp]
│
└─ if (system_core->get_time() >= t_output_next - 1e-6):
       spdlog::info(time, step, stopwatch)
       outputs->output_all(nstep)       ← CSV channels + optional VTK mesh export
       t_output_next += outputs->dt_output
```

Output reads the aero/hydro positions that were just synchronized in poststep. If output were triggered
*before* poststep, it would report stale geometry from the previous step.

---

## Summary: Prestep vs Poststep Data Flow

| Phase | Direction | What is transferred |
|---|---|---|
| Prestep | fluid → elasto | Forces, moments, added-mass matrices |
| Step | (internal) | Chrono advances equations of motion |
| **Poststep** | **elasto → fluid** | **Positions, rotations, velocities, accelerations** |

---

## Component Responsibility Summary

| Class | File | Role in poststep |
|---|---|---|
| `System` | `src/core/system.cpp` | Iterates turbines + standalone components |
| `Turbine` | `src/core/turbine.cpp` | Resets RNA loads, applies gearbox loss, delegates to sub-components |
| `RotorNacelleAssembly` | `src/core/rotor.cpp` | Delegates to rotor, syncs hub/nacelle aero state |
| `Rotor` | `src/core/rotor.cpp` | Iterates blades |
| `Blade` | `src/core/blade.cpp` | `update_positions_aero()` — elasto FEA nodes → aero nodes |
| `Tower` | `src/core/tower.cpp` | `update_positions_aero()` — elasto FEA nodes → aero nodes |
| `Floater` | `src/core/floater.cpp` | Delegates to mooring system, then syncs main body hydro state |
| `Mooring` | `src/core/mooring.cpp` | `update_positions_hydro()` — elasto FEA nodes → hydro nodes |
| `MooringSystem` | `src/core/mooring.cpp` | Iterates all mooring lines |
| `Controller` | `src/servo/controller.cpp` | State save only (no-op base) |
| `ControllerVariableTorque` | `src/servo/controller.cpp` | Stores `torque_elec_previous` |
| `ControllerDISCON` | `src/servo/controller_discon.cpp` | Inherits no-op from base |
