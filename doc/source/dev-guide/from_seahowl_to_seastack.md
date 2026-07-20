# From SEAHOWL to SEA-Stack: Added Mass Treatment

This page documents the differences in how infinite-frequency added mass is handled between SEAHOWL (Chrono v9) and SEA-Stack (Chrono v10), and outlines what would be required to migrate SEAHOWL to the newer approach.

---

## Background

Both SEAHOWL and SEA-Stack rely on Project Chrono as their multibody dynamics backend and use potential-flow BEM data (via WAMIT/NEMOH/Capytaine) for hydrodynamic coefficients. The infinite-frequency added mass matrix — the $A(\infty)$ term from BEM theory — must be incorporated into the system dynamics.

Because Chrono v9 did not natively support dense multi-body added mass assembly, SEAHOWL implemented a workaround via a custom load class. SEA-Stack, targeting Chrono v10, uses Chrono's native `ChLoadHydrodynamics` instead.

---

## SEAHOWL Approach: `ChLoadLocal66`

SEAHOWL defines a custom class `ChLoadLocal66 : public ChLoadCustom` (in `src/elasto/chrono_adapters.cpp`). Each rigid body or FEA node that carries hydrodynamic added mass receives its own instance, registered through a `ChLoadContainer`.

### How it works

- `ComputeJacobian` populates the `m_jacobians->M` field with the rotated 6×6 added mass matrix for that single body.
- `LoadIntLoadResidual_Mv` manually scatters `c * M * v` into the global residual vector for the body's DOFs.
- Frame transformation is handled explicitly: translation DOFs are in the global frame; rotation DOFs in the local body frame. A mixed rotation matrix `rot66` is built and applied:

```cpp
m_jacobians->M = rot66 * (added_mass_matrix * rot66.inverse());
```

- Each body is treated **independently** — one `ChLoadLocal66` instance per body, each with a 6×6 matrix.

### Structural pattern

```
BodyElastoChrono
  └── ChLoadContainer  (chloadcontainer)
        ├── ChLoadForceTorque    (internal forces/moments)
        └── ChLoadLocal66       (added mass + damping, 6×6 per body)
              ├── ComputeQ()          → damping force
              ├── ComputeJacobian()   → M (added mass), R (damping)
              └── LoadIntLoadResidual_Mv()  → c * M * v scatter
```

---

## SEA-Stack Approach: `ChLoadHydrodynamics`

SEA-Stack uses Chrono v10's native `ChLoadHydrodynamics` (in `adapters/chrono/src/hydro_system.cpp`). At setup, the full $6N \times 6N$ infinite-frequency added mass matrix (including cross-body coupling) is registered once with the Chrono system. Chrono assembles it directly into the solver's global mass matrix.

### How it works

```cpp
ChBodyAddedMassBlocks body_blocks;
for (int i = 0; i < num_bodies_; i++) {
    body_blocks.push_back({bodies_[i], body_info[i].inf_added_mass});
}
auto hydro_load = chrono_types::make_shared<::chrono::ChLoadHydrodynamics>(body_blocks);
bodies_[0]->GetSystem()->Add(hydro_load);
```

Each `ChBodyAddedMassBlock` carries a `6 × 6N` row block — body $i$'s six rows against all $6N$ hydro DOFs. Chrono v10 assembles these into the mass matrix at solver setup without requiring a `ChLoadContainer`.

---

## Side-by-Side Comparison

| Aspect | SEAHOWL (`ChLoadLocal66`) | SEA-Stack (`ChLoadHydrodynamics`) |
|--------|--------------------------|----------------------------------|
| Chrono version | v9 | v10 |
| Matrix scope | 6×6 per body (local only) | 6N×6N (global, all bodies) |
| Cross-body coupling | **Not supported** | Fully supported |
| Assembly method | `ChLoadCustom` → Jacobian `M` field | Direct mass-matrix assembly in solver |
| Frame convention | Manual `rot66 * M * rot66⁻¹` | Handled internally by Chrono v10 |
| Lagging risk | Applied through load Jacobian path; coupling is not as tight as direct mass injection | Same-step (no lag) — assembled before solve |
| FEA node support | Yes (`ChNodeFEAxyzrot`) | Rigid bodies only |
| Container required | Yes (`ChLoadContainer`) | No — added directly to `ChSystem` |

---

## Limitations and Known Issues with `ChLoadLocal66`

### 1. No multi-body cross-coupling

BEM solvers produce a full $6N \times 6N$ added mass matrix. The off-diagonal $6 \times 6$ blocks represent hydrodynamic coupling between bodies (e.g., two floating bodies in close proximity). SEAHOWL's per-body `ChLoadLocal66` can only inject the diagonal self-blocks. Cross-coupling terms are silently discarded.

For single-body configurations this is exact. For multi-body floating systems (e.g., arrays, semi-submersibles with multiple pontoons treated as separate BEM bodies) it introduces a modelling error.

### 2. Indirect mass injection

Adding added mass through `ChLoadCustom` + `ComputeJacobian` is a workaround for the absence of native support in Chrono v9. The Jacobian `M` field is used by Chrono's time integrators during iterative solves, but the code path differs from the direct mass-matrix assembly that `ChLoadHydrodynamics` uses. In practice this means:

- The added mass effect is evaluated at each iteration of the nonlinear solver rather than being pre-assembled.
- Errors or inconsistencies can arise depending on solver configuration if the Jacobian update frequency is not at every iteration.

### 3. Manual rotation bookkeeping

Chrono v9 sends translation velocity components in the global frame and rotational components in the local body frame. `ChLoadLocal66` compensates with the mixed `rot66` transform. This is correct for small angles and well-tested, but it is an additional source of complexity and potential error compared to Chrono v10's native handling.

---

## What It Would Take to Migrate SEAHOWL

The following steps outline a migration path. These are described at a high level; each step has its own scope and risk profile.

### Step 1 — Upgrade to Chrono v10 (prerequisite)

`ChLoadHydrodynamics` does not exist in Chrono v9. A Chrono version upgrade is the hard prerequisite. This carries a wider migration cost:

- Chrono v10 introduced API changes across `ChBody`, FEA, solvers, and contact models.
- The full scope of breaking changes affecting SEAHOWL must be audited before starting.

### Step 2 — Replace `ChLoadLocal66` with `ChLoadHydrodynamics` for rigid bodies

For rigid bodies carrying BEM hydrodynamic loads, remove the per-body `ChLoadLocal66` instances and replace with a single `ChLoadHydrodynamics` registration at the system level (following the SEA-Stack pattern in `HydroSystem`).

This requires:

- Supplying the full $6N \times 6N$ `inf_added_mass` matrix (likely already available from the H5/BEM data source).
- Removing the manual `rot66` transform — Chrono v10 handles frame conventions internally.
- Removing the `ChLoadContainer` registration for the added mass path (keep it for other loads if still needed).

### Step 3 — Retain a per-node mechanism for FEA nodes

`ChLoadHydrodynamics` only supports `ChBody`. SEAHOWL uses `ChLoadLocal66` for `ChNodeFEAxyzrot` nodes as well (distributed Morison added mass along flexible members). This path must be preserved.

Options:
- Keep `ChLoadLocal66` for FEA nodes only, updating it to compile against Chrono v10 APIs.
- Use an equivalent Chrono v10 custom load pattern for distributed node loads.

### Step 4 — Provide the full coupled matrix

The SEAHOWL hydro module must supply the full $6N \times 6N$ `inf_added_mass` matrix including cross-coupling blocks. If the current BEM data pipeline already stores this (e.g., from a BEMIO HDF5 file with multi-body export), this is a data routing task. If only per-body 6×6 blocks are stored, the import pipeline must be extended.

### Step 5 — Regression testing

After migration, verify against reference results for:

- Single-body cases (should be unchanged).
- Multi-body cases (cross-coupling now active — expect differences from previous results if off-diagonal terms are non-zero).

---

## Coordinate Frame Transformations for the Added Mass Matrix

This section traces every transformation applied to `added_mass_matrix` as it travels from OpenFAST/HydroDyn to Chrono's solver, explaining why only a pre-multiplication appears at line 433 of `hydrodyn_adapter.cpp`.

### Motivation

OpenFAST and Chrono use different coordinate conventions for angular kinematics:

- **OpenFAST/HydroDyn**: all nodal kinematics (velocity, acceleration) are expressed in the **global/inertial frame**, including angular components.
- **Chrono v9 (`ChLoadCustom`)**: the state-delta vector `w` that Chrono passes into `LoadIntLoadResidual_Mv` and `ComputeJacobian` is **mixed** — translational DOFs in the global frame, rotational DOFs in the **local body frame**.

The added mass relation is $F = A \cdot \ddot{q}$. Changing the frame in which $\ddot{q}$ is expressed requires a change-of-basis on the right (post-multiplication), and changing the frame in which $F$ is expressed requires a change-of-basis on the left (pre-multiplication). The two-stage transform applied across `hydrodyn_adapter.cpp` and `ChLoadLocal66` exploits this split.

### Stage-by-Stage Trace

#### Stage 1 — HydroDyn output (`NodeAdm`)

**Location:** `HydroDynAdapter::compute_loads` → `interface_hydrodyn->NodeAdm`

HydroDyn outputs the added mass matrix in the **fully global** frame. Both the force output and the acceleration input of $A$ are expressed in the global/inertial frame for all six DOFs:

$$A_{global}: \quad F_{global} = A_{global} \cdot \ddot{q}_{global}$$

This is confirmed by `update_nodes_motion` where angular velocity and acceleration are passed with the `false` flag (global frame):

```cpp
auto node_rotvel = node.get_rotational_velocity(false);   // global frame
auto node_rotacc = node.get_rotational_acceleration(false); // global frame
```

#### Stage 2 — Pre-multiplication in `FloaterHydroDyn::compute_env_loads` (line 433)

**Location:** `src/fluid/hydro/hydrodyn_adapter.cpp`, line 433

```cpp
Eigen::Matrix<double, 6, 6> rot66 = Eigen::Matrix<double, 6, 6>::Zero();
Eigen::Matrix<double, 3, 3> rot33(body->get_rotation_matrix());
rot66.block<3, 3>(0, 0) = rot33;  // full rotation for translation block
rot66.block<3, 3>(3, 3) = rot33;  // full rotation for rotation block

added_mass_matrix = rot66.inverse() * added_mass_matrix;
```

Note that `rot66` is the **full** rotation (body rotation matrix in both the 3×3 translational and 3×3 rotational sub-blocks).

This transforms only the **output side** (force) from global to local:

$$A_{local \leftarrow global} = R_{full}^{-1} \cdot A_{global}$$

After this step: $F_{local} = A_{local \leftarrow global} \cdot \ddot{q}_{global}$

The acceleration input side is still global. This is a deliberate intermediate state — the matrix is not yet in its final frame.

The identical logic is applied to monopile nodes at line 489.

#### Stage 3 — Storage in `ChLoadLocal66`

**Location:** `BodyElastoChrono::set_added_mass_matrix` → `chload66->SetAddedMassMatrix(matrix)`

For the **floater body** path, the matrix is passed directly to `ChLoadLocal66` with no additional conversion. For **FEA nodes** (`NodeElastoChrono::set_added_mass_matrix`), an additional IEC↔Chrono axis convention rotation is applied:

```cpp
// IEC: x=flapwise, y=edgewise, z=longitudinal
// Chrono node: x=longitudinal, y=edgewise, z=flapwise
auto mm = rot66_iec2ch * (matrix * rot66_iec2ch.transpose());
chload66->SetAddedMassMatrix(mm);
```

This is a full similarity transform for the FEA node axis swap. It is not needed on rigid bodies because the floater uses the global frame directly.

#### Stage 4 — Final transform in `ChLoadLocal66::ComputeJacobian` (line 202)

**Location:** `src/elasto/chrono_adapters.cpp`, line 202

```cpp
// Chrono sends:
// - translation components in global system
// - rotation components in local system
Eigen::Matrix<double, 6, 6> rot66 = Eigen::Matrix<double, 6, 6>::Zero();
rot66.block<3, 3>(0, 0) = rot33;  // body rotation (translation block)
rot66.block<3, 3>(3, 3) = I;      // IDENTITY (rotation block)

m_jacobians->M = rot66 * (added_mass_matrix * rot66.inverse());
```

This `rot66` is the **mixed-frame** rotation — body rotation for translational DOFs, identity for rotational DOFs. It applies a full similarity transform to the stored matrix.

The combined effect of stages 2 and 4 for the floater body is:

$$M_{chrono} = R_{mixed} \cdot \left( R_{full}^{-1} \cdot A_{global} \right) \cdot R_{mixed}^{-1}$$

Expanding $R_{mixed}$ (translation: $R$, rotation: $I$) and $R_{full}$ (both: $R$):

- The **translational** rows/columns: $R \cdot R^{-1} \cdot A_{tt} \cdot R \cdot R^{-1} = A_{tt}$ — the translational self-block passes through unchanged.
- The **translational-rotational** coupling blocks: the combined transform correctly maps the off-diagonal coupling from full-global to Chrono's mixed convention.
- The **rotational** rows/columns: $I \cdot R^{-1} \cdot A_{rr} \cdot R \cdot I^{-1} = R^{-1} \cdot A_{rr} \cdot R$ — these end up in the local body frame, matching Chrono's local-frame rotational DOFs.

### Why Only One-Sided at Line 433?

The one-sided pre-multiplication at line 433 is **intentional and correct**. It would be wrong to apply the full similarity transform $R^{-1} A R$ there, because `ChLoadLocal66` later applies its own `rot66 * M * rot66.inverse()` using a *different* (mixed) `rot66`. Applying a full transform at line 433 would double-transform the rotational blocks. The two-stage design distributes the frame conversion across two sites:

| Site | Operation | Purpose |
|------|-----------|---------|
| `hydrodyn_adapter.cpp` line 433 | $R_{full}^{-1} \cdot A$ | Rotate force output to local frame |
| `chrono_adapters.cpp` line 202 | $R_{mixed} \cdot M \cdot R_{mixed}^{-1}$ | Rotate acceleration input to Chrono mixed frame |

### Fragility Note

This correctness depends on the two `rot66` constructions being **different** — full rotation at line 433, mixed rotation at line 202. This is non-obvious and easy to break if someone modifies one site without understanding the other. If both were made identical (e.g., both full or both mixed), the rotational block would be incorrectly transformed.

This coupling between two distant code sites is one of the reasons the SEA-Stack approach with `ChLoadHydrodynamics` (which internalises all frame handling in Chrono v10) is cleaner — the user simply passes the added mass in the BEM/body frame and Chrono manages the rest.

---

## Relation to SEA-Stack Architecture

SEA-Stack's approach is documented in detail in the SEA-Stack `TECHNICAL_OVERVIEW.md`, Appendix F.3.1. The key design principle is:

> The infinite-frequency added-mass matrix from the BEM data is passed directly into Chrono, where it is added to the system mass matrix. This includes cross-body coupling when present in the data. Chrono v10 supports this dense added-mass coupling directly (via `ChLoadHydrodynamics`). Other solvers would need equivalent support, or a different approach, to reproduce the same behaviour.

For SEAHOWL, the adapter layer equivalent would be the `BodyElastoChrono` / hydro coupling code in `src/elasto/chrono_adapters.cpp` and the relevant fluid module.

---

## Open Questions

- Does SEAHOWL currently run any multi-body hydrodynamic cases where cross-coupling is present in the BEM data? If yes, this is an active modelling error; if no, the issue is latent.
- What is the full scope of Chrono v9 → v10 API breaks in SEAHOWL beyond the added mass path?
- Is there a plan to consolidate SEAHOWL's hydro interface with SEA-Stack's `HydroSystem`/`HydroForces` domain layer, or will they remain separate codebases?
