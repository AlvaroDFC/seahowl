# SEA-Stack ↔ Chrono Integration

This page documents how SEA-Stack connects its hydrodynamic, PTO, and mooring modules to the
Project Chrono multibody solver, and how body state flows back from Chrono into those modules
during the simulation loop.

It complements [From SEAHOWL to SEA-Stack](from_seahowl_to_seastack.md) and the
[SEAHOWL ↔ Chrono Interface](architecture/seahowl_chrono_interface.md) page.

---

## Key Architectural Difference from SEAHOWL

SEAHOWL uses an explicit **prestep / step / poststep** orchestrator pattern:

```
System::prestep(time, dt)   ← fluid loads evaluated, transferred to elasto nodes
system.DoStepDynamics(dt)   ← Chrono integration
System::poststep(time, dt)  ← state synchronisation back to fluid / servo modules
```

SEA-Stack does **not** use this pattern. Instead, all force contributions are registered
ahead of time as Chrono objects (`ChForce`, `ChLoad`, `ChLinkTSDA`). Chrono owns the
integration loop and **pulls** forces on demand during its own internal force-assembly
phase inside `DoStepDynamics`. The application-level loop is therefore minimal:

```cpp
// apps/seastack/single_run.cpp — the entire per-step application code
system->DoStepDynamics(loop_dt);   // Chrono drives everything internally
exporter->RecordStep(system.get()); // post-step output capture only
```

---

## How Forces Enter the Chrono System

### 1 — Hydrodynamic forces (hydrostatics + radiation + excitation + mooring)

**Mechanism:** `ChBody::AddForce(ChForce)` with a `ChFunction` callback.

#### The `ComponentFunc` subclass

`ComponentFunc` is a SEA-Stack class that **inherits from Chrono's `ChFunction`** base class
and overrides its single key virtual method:

```cpp
// adapters/chrono/src/chrono_force_attacher.cpp (implementation detail, not in public API)
class ComponentFunc : public ::chrono::ChFunction {
    double GetVal(double /*time*/) const override {
        return attacher_->CoordinateFuncForBody(body_num_, dof_index_);
    }
};
```

Chrono knows nothing about `HydroSystem` or SEA-Stack — it only sees a `ChFunction*`
and calls `GetVal(time)` on it during its internal force-assembly phase. This is standard
Chrono usage; SEA-Stack simply places its own physics behind that virtual call.

At construction time (`ChronoForceAttacher` ctor), two `ChForce` objects are attached to each
hydrodynamic body: one for the force vector (Fx, Fy, Fz) and one for the torque vector
(Tx, Ty, Tz). Each component is driven by a `ComponentFunc` instance:

```
ChForce (hydroforce)   → SetF_x(ComponentFunc(body, 0))
                       → SetF_y(ComponentFunc(body, 1))
                       → SetF_z(ComponentFunc(body, 2))

ChForce (hydrotorque)  → SetF_x(ComponentFunc(body, 3))
                       → SetF_y(ComponentFunc(body, 4))
                       → SetF_z(ComponentFunc(body, 5))
```

During Chrono's internal force assembly, `ComponentFunc::GetVal(time)` is called once per DOF.
The first call for a new timestep triggers a full hydro evaluation; subsequent calls for the
remaining DOFs of the same step return cached values.

#### `CoordinateFuncForBody` — the Chrono / SEA-Stack boundary

`CoordinateFuncForBody` is the single crossing point between Chrono's world and SEA-Stack's
physics. On entry it checks whether the timestep has advanced; if so it reads body state from
Chrono, calls the `force_evaluator_` lambda (which leads to `HydroSystem`), and caches the
result. If the timestep has not changed, it returns the cached value immediately:

```cpp
// Simplified logic inside CoordinateFuncForBody
if (bodies_[0]->GetChTime() == prev_time_) {
    return total_force_[body_num_offset + dof_index];  // cache hit — no re-evaluation
}
prev_time_ = bodies_[0]->GetChTime();
BuildSystemStateFromChronoBodies(bodies_, cached_state_);  // pull state from Chrono
total_force_ = force_evaluator_(prev_time_);               // run all SEA-Stack hydro physics
return total_force_[body_num_offset + dof_index];
```

The `force_evaluator_` is a `std::function` lambda wired at construction time in `HydroSystem`:

```cpp
// adapters/chrono/src/hydro_system.cpp
auto evaluator = [this](double time) -> seastack::hydro::BodyForces {
    return EvaluateForces(time);
};
force_attacher_ = std::make_unique<ChronoForceAttacher>(
    bodies_, std::move(evaluator), ...);
```

Because Chrono calls `GetVal` once per DOF (6 per body), and there are N bodies, the cache
ensures `HydroForces::Evaluate` runs exactly **once per timestep**, regardless of body count.

```
ComponentFunc::GetVal(time)                     ← Chrono virtual dispatch
  └─ ChronoForceAttacher::CoordinateFuncForBody(body, dof)   ← boundary crossing
      ├─ [cache hit]  return total_force_[offset + dof]
      └─ [new step]
          BuildSystemStateFromChronoBodies()    ← read pos/vel/rot from ChBody
          force_evaluator_(time)                ← lambda → HydroSystem::EvaluateForces
                └─ ChronoHydroCoupler::Evaluate(time)
                      └─ HydroForces::Evaluate(state, time)  ← Chrono-free
                            ├─ HydrostaticsComponent::Evaluate()
                            ├─ RadiationComponent::Evaluate()
                            ├─ ExcitationComponent::Evaluate()
                            └─ MooringComponent::Evaluate()  ← if MoorDyn enabled
```

**Relevant files:**

| File | Purpose |
|------|---------|
| [adapters/chrono/src/chrono_force_attacher.cpp](../../../../sea-stack/adapters/chrono/src/chrono_force_attacher.cpp) | `ComponentFunc`, `ChronoForceAttacher` — force registration and per-step caching |
| [adapters/chrono/src/hydro_system.cpp](../../../../sea-stack/adapters/chrono/src/hydro_system.cpp) | `HydroSystem::EvaluateForces`, lazy model construction |
| [adapters/chrono/src/chrono_hydro_coupler.cpp](../../../../sea-stack/adapters/chrono/src/chrono_hydro_coupler.cpp) | `ChronoHydroCoupler::Evaluate` — bridges Chrono state to `HydroForces` |
| [adapters/chrono/src/chrono_state_utils.cpp](../../../../sea-stack/adapters/chrono/src/chrono_state_utils.cpp) | `BuildSystemStateFromChronoBodies` — state extraction |

### 2 — Infinite-frequency added mass

**Mechanism:** `ChSystem::Add(ChLoadHydrodynamics)` — registered once at construction, not a runtime callback.

Chrono v10 natively supports dense multi-body added mass assembly through `ChLoadHydrodynamics`.
SEA-Stack passes the $A(\infty)$ matrix from the BEM data (read from HDF5) directly to this load
object, which Chrono incorporates into the mass matrix during integration. No per-step application
code is needed.

```
HydroSystem ctor
  └─ chrono_types::make_shared<ChLoadHydrodynamics>(body_blocks)
       └─ bodies_[0]->GetSystem()->Add(hydro_load)
```

**File:** [adapters/chrono/src/hydro_system.cpp](../../../../sea-stack/adapters/chrono/src/hydro_system.cpp) (lines ~80–92)

!!! note "Contrast with SEAHOWL"
    SEAHOWL uses a custom `ChLoadLocal66 : public ChLoadCustom` per body because Chrono v9 lacked
    native multi-body added mass support. SEA-Stack replaces this with the Chrono v10 built-in.
    See [From SEAHOWL to SEA-Stack](from_seahowl_to_seastack.md) for the full comparison.

### 3 — PTO forces (translational and rotational)

**Mechanism:** `ChLinkTSDA::RegisterForceFunctor(PTOForceFunctor)` / `ChLinkRSDA` equivalent.

The `PTOForceFunctor` wraps any solver-agnostic `seastack::pto::IPTOModel` into Chrono's
spring-damper-actuator link interface. Chrono evaluates the functor each step when resolving
the TSDA/RSDA constraint, passing current length, velocity, and rest length:

```cpp
double PTOForceFunctor::evaluate(double time, double rest_length,
                                 double length, double vel,
                                 const ChLinkTSDA& link) {
    double displacement = length - rest_length;
    return model_->ComputeForce(displacement, vel, time);
}
```

The `IPTOModel` implementations (linear, hydraulic, etc.) live in `libs/pto/` with **no
Chrono dependency** — the adapter in `adapters/chrono/` is the only coupling point.

**File:** [adapters/chrono/src/pto_chrono_adapter.cpp](../../../../sea-stack/adapters/chrono/src/pto_chrono_adapter.cpp)

---

## Dependency Isolation Design

A deliberate architectural rule governs the boundary between SEA-Stack and Chrono:

```
┌────────────────────────────────────────────────────────────┐
│  libs/hydro/      libs/pto/      libs/mooring/             │
│  (zero Chrono dependency — plain Eigen, no ChBody)         │
└──────────────────────────┬─────────────────────────────────┘
                           │  SystemState (Eigen vectors only)
┌──────────────────────────▼─────────────────────────────────┐
│  adapters/chrono/                                          │
│  ChronoForceAttacher  ChronoHydroCoupler  chrono_state_utils│
│  PTOForceFunctor      HydroSystem         setup_from_yaml  │
│  (all Chrono knowledge is contained here)                  │
└────────────────────────────────────────────────────────────┘
```

- `HydroForces::Evaluate` receives only a `SystemState` struct (plain Eigen vectors for
  position, velocity, orientation, angular velocity). It has no `#include` of any Chrono header.
- All `ChBody`, `ChForce`, `ChFunction`, and `ChSystem` usage is confined to `adapters/chrono/`.
- This means the entire hydrodynamics stack can be unit-tested without linking Chrono, and
  the adapter layer can be swapped for a different MBD solver without touching `libs/hydro/`.

The three crossing points where Chrono types are converted to/from SEA-Stack types are:

| Crossing point | Direction | Where |
|---|---|---|
| `BuildSystemStateFromChronoBodies` | Chrono → SEA-Stack (state in) | `chrono_state_utils.cpp` |
| `ComponentFunc::GetVal` / `ChronoForceAttacher` | SEA-Stack → Chrono (force values out) | `chrono_force_attacher.cpp` |
| `PTOForceFunctor::evaluate` | SEA-Stack → Chrono (TSDA force out) | `pto_chrono_adapter.cpp` |

---

## How Body State Flows Back from Chrono

SEA-Stack does not have a dedicated poststep. Body state is read back through two separate paths:

### A — During force evaluation (within DoStepDynamics)

`BuildSystemStateFromChronoBodies` is called by `CoordinateFuncForBody` on every new timestep,
converting Chrono body data into a `SystemState` struct consumed by `HydroForces::Evaluate`:

| Chrono API | `SystemState` field |
|------------|---------------------|
| `body->GetPos().eigen()` | `body_state.position` |
| `body->GetRot().GetCardanAnglesXYZ().eigen()` | `body_state.orientation_rpy` |
| `body->GetPosDt().eigen()` | `body_state.linear_velocity` |
| `body->GetAngVelParent().eigen()` | `body_state.angular_velocity` |

**File:** [adapters/chrono/src/chrono_state_utils.cpp](../../../../sea-stack/adapters/chrono/src/chrono_state_utils.cpp)

### B — After DoStepDynamics (output recording)

`SimulationExporter::RecordStep` iterates `system->GetBodies()` and `system->GetLinks()` to
capture the full state snapshot for HDF5 output. TSDA/RSDA link force and velocity are also
read here for PTO power and energy accounting:

| Chrono API | Recorded quantity |
|------------|------------------|
| `body->GetPos()`, `GetPosDt()`, `GetPosDt2()` | position, velocity, acceleration |
| `body->GetRot()` | quaternion, Euler XYZ |
| `body->GetAngVelParent()` | angular velocity |
| `ChLinkTSDA::GetVelocity()`, `GetForce()`, `GetExtension()` | PTO speed, force, stroke |
| `ChLinkTSDA::GetForceFunctor()` (→ `PTOForceFunctor`) | selects $-(F \cdot v)$ power formula |

**File:** [adapters/chrono/src/simulation_export.cpp](../../../../sea-stack/adapters/chrono/src/simulation_export.cpp)

---

## Full Timing Diagram

```mermaid
sequenceDiagram
    participant Loop as single_run.cpp
    participant Chrono as ChSystem
    participant CF as ComponentFunc
    participant FA as ChronoForceAttacher
    participant SS as chrono_state_utils
    participant Coupler as ChronoHydroCoupler
    participant Hydro as HydroForces::Evaluate
    participant Exp as SimulationExporter

    Loop->>Chrono: DoStepDynamics(dt)
    Note over Chrono: internal force assembly
    Chrono->>CF: GetVal(time) [×6 per body]
    CF->>FA: CoordinateFuncForBody(body, dof)
    FA->>SS: BuildSystemStateFromChronoBodies
    SS-->>FA: SystemState (pos, vel, rot, angvel)
    FA->>Coupler: EvaluateForces(time)
    Coupler->>Hydro: Evaluate(state, time)
    Hydro-->>Coupler: BodyForces (hydrostatics+radiation+excitation+mooring)
    Coupler-->>FA: BodyForces
    FA-->>CF: force[dof] (cached for remaining DOFs)
    CF-->>Chrono: scalar force value
    Note over Chrono: TSDA functor called for PTO links
    Chrono-->>Loop: (step complete)
    Loop->>Exp: RecordStep(system)
    Exp->>Chrono: GetBodies(), GetLinks()
    Chrono-->>Exp: body states + TSDA/RSDA values
```

---

## Summary Table

| # | Contribution | Entry mechanism | Trigger point | Key files |
|---|-------------|----------------|---------------|-----------|
| 1 | Hydrostatics + radiation + excitation | `ChForce` + `ComponentFunc` callback | Chrono force assembly (inside `DoStepDynamics`) | `chrono_force_attacher.cpp`, `hydro_system.cpp` |
| 2 | Infinite-frequency added mass $A(\infty)$ | `ChLoadHydrodynamics` (native Chrono v10) | Added to mass matrix at construction | `hydro_system.cpp` |
| 3 | PTO damping / spring / actuator | `ChLinkTSDA` + `PTOForceFunctor` | Chrono constraint solver (inside `DoStepDynamics`) | `pto_chrono_adapter.cpp` |
| 4 | Mooring (MoorDyn) | Bundled into `HydroForces::Evaluate` as `MooringComponent` | Same as row 1 (via hydro callback) | `hydro_system.cpp`, `libs/mooring/` |
| 5 | State pull (pos/vel/rot) for force eval | `BuildSystemStateFromChronoBodies` | First DOF query per timestep in `CoordinateFuncForBody` | `chrono_state_utils.cpp` |
| 6 | State capture for HDF5 output | `SimulationExporter::RecordStep` | After `DoStepDynamics` returns | `simulation_export.cpp` |
