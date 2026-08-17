# AeroDyn–Chrono preCICE Adapter

## Architecture and Implementation Plan

### Document status

This document consolidates the design decisions reached for a standalone
preCICE adapter between Project Chrono and the OpenFAST AeroDyn–InflowWind
combined C binding, referred to below as ADI.

The implementation is intentionally staged:

| Item | Decision |
|---|---|
| Primary application | Vertical-axis wind turbines (VAWTs) |
| Required compatibility | Horizontal-axis wind turbines (HAWTs) |
| First runnable structural model | Rigid blades |
| Intended structural expansion | Flexible, torsion-capable beams using ChNodeFEAxyzrot nodes |
| Initial coupling scheme | Explicit coupling |
| AeroDyn interface | ADI_C_* C-binding calls, not the complete OpenFAST executable |
| Input strategy | A small, versioned, paths-only coupling YAML plus normal AeroDyn and InflowWind input files |
| Structural geometry owner | Chrono |

The code should be designed around the flexible-blade data contract from the
start, even though the first end-to-end case will use rigid blades. The rigid
implementation is therefore a motion-provider implementation, not a separate
adapter architecture that will later have to be replaced.

---

## 1. General goal

The goal is to run a two-participant co-simulation in which:

1. Chrono advances the structural model.
2. Chrono sends rotor and blade kinematics through preCICE.
3. An AeroDyn participant converts those kinematics to the ADI C-binding
   representation.
4. AeroDyn and InflowWind compute aerodynamic loads.
5. The AeroDyn participant returns nodal forces and moments through preCICE.
6. Chrono applies those loads to the rigid or flexible blade model.

The first accepted end-to-end demonstration is one rigid-bladed VAWT. The
schema, typed data model, mesh organization, and ADI marshalling code must also
support a HAWT and must not assume a horizontal rotor axis.

### 1.1 In scope

- Direct use of AeroDyn and InflowWind through the ADI_C_* API.
- Parsing a dedicated AeroDyn–preCICE coupling YAML file.
- Loading AeroDyn.dat and InflowWind.dat from paths.
- A VAWT-first rotor description with explicit HAWT compatibility.
- Rigid-blade coupling as the first implementation.
- A solver-neutral blade-node contract that can later be populated from
  ChNodeFEAxyzrot nodes.
- Forces and moments returned at coupling stations.
- One turbine for the first integration case, while retaining the ADI
  multi-turbine call structure.
- Fixed-step, explicit preCICE coupling for version 1.

### 1.2 Out of scope for version 1

- Calling or parsing the complete OpenFAST driver input chain.
- Reimplementing the OpenFAST YAML readers in this project.
- Inline AeroDyn or InflowWind input mappings inside the coupling YAML.
- HydroDyn, SeaState, ServoDyn, MoorDyn, or a general OpenFAST orchestrator.
- Implicit coupling without a proven AeroDyn state rollback mechanism.
- ANCF beam formulations.
- Multiple MPI ranks per participant.
- A general remeshing or topology-changing interface.

HydroDyn and other OpenFAST modules may be addressed by separate plans later.
They are intentionally removed from this document so that the current work has
one clear solver boundary.

### 1.3 Upstream work versus adapter work

The project should distinguish dependencies from work it owns.

| Topic | Classification | Action in this project |
|---|---|---|
| OpenFAST full-driver YAML chain | Upstream development | Track the format; do not implement it here |
| ADI mapping of HAWT and VAWT to AeroProjMod | Upstream behavior | Document and regression-test the selected behavior |
| AeroDyn or InflowWind physics and input validation | Upstream behavior | Pass files to ADI and report its diagnostics |
| preCICE direct access to received meshes | External API capability | Use it through the adapter lifecycle |
| Chrono rigid-point rotation and angular-velocity exchange | Adapter integration work | Implement or extend the Chrono participant |
| Chrono FEA_MESH_NODES behavior | Not wired in the inspected feature branch | Implement in the flexible-blade phase |
| Typed configuration, marshalling, ADI lifecycle, and tests | Project work | Implement and maintain here |

### 1.4 Reference snapshot

The plan is based on the following source snapshots and should record exact
commit IDs in the build once implementation begins:

- [OpenFAST YAML-input development branch](https://github.com/mayankchetan/openfast/tree/f/yaml-input)
- [FAST_Yaml.f90 module-file handling](https://github.com/mayankchetan/openfast/blob/f/yaml-input/modules/openfast-library/src/FAST_Yaml.f90)
- [OpenFAST ADI C-binding implementation](https://github.com/OpenFAST/openfast/blob/main/modules/aerodyn/src/AeroDyn_Inflow_C_Binding.f90)
- [OpenFAST ADI Python wrapper and usage sequence](https://github.com/OpenFAST/openfast/blob/main/glue-codes/python/pyOpenFAST/aerodyn_inflow.py)
- [Chrono preCICE development branch](https://github.com/projectchrono/chrono/tree/feature/precice)
- [Chrono preCICE MBS adapter](https://github.com/projectchrono/chrono/blob/feature/precice/src/chrono_precice/ChPreciceAdapterMbs.cpp)
- [Chrono tapered Timoshenko beam using ChNodeFEAxyzrot](https://github.com/projectchrono/chrono/blob/main/src/chrono/fea/ChElementBeamTaperedTimoshenko.h)
- [preCICE direct access to received meshes](https://precice.org/couple-your-code-direct-access)

The local baseline files AeroDynCBinding.h, AeroDynPreciceAdapter.h/.cpp,
OfPreciceAdapter.h/.cpp, main.cpp, and CMakeLists.txt remain the immediate
implementation starting point.

---

## 2. System architecture

There are two independent solver processes:

- The Chrono participant owns the structural model and the coupling meshes.
- The AeroDyn participant owns the ADI library instance and aerodynamic state.

preCICE transports data between them. The two participants do not share Chrono
objects, ADI internal objects, or configuration structures.

| Direction | Information |
|---|---|
| Chrono → AeroDyn | Initial mesh coordinates; positions; orientations; linear velocities; angular velocities |
| AeroDyn → Chrono | Forces; moments |

Accelerations are accepted by ADI_C_SetRotorMotion, but they are not part of
the version 1 preCICE contract. Version 1 will initialize the corresponding ADI
buffers to zero and validate that assumption against a reference ADI case.
Acceleration exchange can be added later without changing the position,
orientation, velocity, force, or moment contracts.

### 2.1 Single source of truth

Each file owns one category of information:

| Source | Owns |
|---|---|
| Chrono model input | Structural topology, rigid bodies or FEA nodes, reference frames, initial structural positions and orientations, mass and stiffness |
| AeroDyn.dat and its referenced blade/airfoil files | Aerodynamic model, aerodynamic blade definition, airfoils, wake/unsteady-aero settings, and AeroDyn outputs |
| InflowWind.dat | Wind or inflow model |
| AeroDyn coupling YAML | Paths, ADI runtime settings, rotor type, turbine-to-mesh bindings, preCICE data names, and output behavior |
| preCICE XML | Participants, mesh ownership, data fields, coupling scheme, and mapping configuration |

The coupling YAML must not contain copies of blade-node coordinates,
orientations, hub positions, nacelle positions, or blade-root positions.
Chrono publishes those values through preCICE.

The structural mesh and aerodynamic discretization are different physical
representations and need not have identical node counts. AeroDyn already uses
its mesh-mapping machinery between the external structural points passed to
ADI_C_SetupRotor and its aerodynamic blade meshes. Checks should therefore
enforce shared invariants, not falsely require both discretizations to be the
same mesh.

Shared invariants include:

- turbine and blade count;
- one and only one blade assignment for every coupling point;
- deterministic reference-frame roles;
- consistent units;
- consistent global axes and handedness;
- a common turbine origin;
- valid initial orientation matrices;
- stable point counts for the entire run.

If an invariant cannot be inspected through the current ADI API, the adapter
must say that it is not machine-verifiable. It must not silently claim that two
definitions are equivalent.

### 2.2 Preferred mesh ownership

Chrono should own the reference-frame and blade coupling meshes. The AeroDyn
participant should use preCICE direct access to the received meshes.

The preCICE configuration therefore gives the AeroDyn participant API access to
the Chrono-owned meshes. The AeroDyn lifecycle then:

1. declares the received meshes and any access region before preCICE
   initialization;
2. calls preCICE initialize;
3. retrieves vertex IDs and coordinates with
   getMeshVertexSize and getMeshVertexIDsAndCoordinates;
4. reads the initial kinematic data;
5. builds the typed rotor description;
6. calls ADI_C_PreInit, ADI_C_SetupRotor, and ADI_C_Init.

This lifecycle avoids a second geometry file. It requires refactoring the
current OfPreciceAdapter lifecycle, which initializes ADI and registers
AeroDyn-owned meshes before preCICE initialize.

If direct mesh access proves unusable in a supported configuration, the
fallback may accept duplicated initialization geometry only with strict
position and orientation tolerance checks. That is a fallback, not the normal
version 1 design.

---

## 3. Current capability and flexible-blade target

The Chrono feature branch exposes these mesh-type names:

- RIGID_BODY_REFS
- RIGID_BODY_POINTS
- FEA_MESH_NODES
- FEA_MESH_POINTS

The enum values alone do not imply complete behavior. In the inspected Chrono
preCICE implementation, FEA mesh loading, registration, reading, and writing
still contain TODO paths, and initialization rejects FEA mesh types.

The implementation status should therefore be stated as follows:

| Capability | Current milestone | Target architecture |
|---|---|---|
| Blade structure | Rigid bodies and body-fixed coupling stations | Flexible beam nodes |
| Blade-node provider | Frame-aware rigid-body points | ChNodeFEAxyzrot nodes |
| Node kinematics | Position, orientation, linear velocity, angular velocity | Same fields |
| Returned load | Nodal force and moment applied to owning rigid body | Nodal force and moment applied to FEA node |
| AeroDyn marshalling | BladeNodeKinematics → ADI arrays | Unchanged |
| Chrono adapter work | Extend rigid points to publish attached frames | Implement FEA_MESH_NODES paths |

This separation is important: the ADI driver and marshaller should not know
whether kinematics came from a rigid body or an FEA node.

---

## 4. Clarifications and decisions reached

### 4.1 Summary of resolved questions

| Question | Decision | Consequence |
|---|---|---|
| Should this consume the full OpenFAST YAML chain? | No | Parse a small coupling manifest and pass module file paths to ADI |
| Should AeroDyn.dat or InflowWind.dat be embedded as YAML mappings? | No in version 1 | Require scalar file paths and reject mappings |
| Is the main use case VAWT-only? | VAWT-first, not VAWT-only | Use a rotor-type enum and keep all mesh logic axis-neutral |
| Who owns structural turbine geometry? | Chrono | The AeroDyn YAML contains mesh bindings, not coordinates |
| Must Chrono and AeroDyn meshes be identical? | No | Validate shared frames and assignments; allow ADI mesh mapping |
| What does a dedicated marshalling function mean? | One component alone flattens typed positions, DCMs, velocities, and indices for ADI | Parsing and preCICE code never construct raw ABI arrays |
| How should DCMs be packed? | C++ row-major buffers with TransposeDCM fixed to 1 | Make this a tested internal contract, not a user option |
| Are ADI motion positions absolute? | No; they are relative to TurbOrigin_C | Subtract the Chrono global turbine origin before every setup and motion call |
| Is a second Chrono input parse needed for origin subtraction? | No | Receive the Chrono-owned origin through the reference mesh and subtract it |
| Can rigid points have orientation and angular velocity? | Physically yes, when an attached point frame is defined | Extend the Chrono rigid-point path to publish the inherited frame motion |
| Are Chrono FEA enum entries already operational? | No | Keep the first milestone rigid; implement FEA wiring as a later phase |
| Can blade torsion be represented by ChNodeFEAxyzrot? | Yes, with a compatible torsion-capable beam and section model | Use it as the first flexible formulation |
| Is the ANCF limitation caused by AeroDyn-relative coordinates? | No | It is caused by how orientation and angular velocity are derived from ANCF gradient coordinates |
| Should returned loads be point or distributed loads? | Point loads in version 1 | Set PointLoadOutput to 1 and exchange force plus moment |
| Should MHK be represented by a Boolean? | No | Use none, fixed-bottom, and floating modes matching the ADI integer values |
| Can checkpoint methods remain empty? | Only for explicit coupling | Reject an implicit configuration until rollback semantics are implemented and tested |

### 4.2 What dedicated marshalling means

The application should use readable, typed values internally:

| Typed value | Example contents |
|---|---|
| ReferenceFrameKinematics | position, orientation, linear velocity, angular velocity |
| BladeNodeKinematics | the same fields for one coupling station |
| BladeNodeLoad | force and moment |
| RotorReferenceState | origin, hub, nacelle, and blade-root frames |

Only AdiMarshaller converts those types to the contiguous buffers required by
the C API:

- positions as x0, y0, z0, x1, y1, z1, and so on;
- one nine-value DCM per frame;
- velocities as translational xyz followed by rotational xyz;
- accelerations in the corresponding six-value buffer;
- forces as force xyz followed by moment xyz;
- one-based blade numbers for MeshPtToBladeNum_C;
- one-based turbine indices for the current Fortran ADI implementation;
- float and double conversions required by the ABI.

This boundary also owns buffer length checks, finite-value checks, stable
storage, and pointer lifetime. The YAML parser and preCICE participant should
never calculate flat-array offsets.

### 4.3 DCM storage and TransposeDCM

A direction cosine matrix is a 3 × 3 orientation matrix. Two separate questions
must be tested:

1. What physical mapping does the matrix represent, such as local-to-global or
   global-to-local?
2. In what order are its nine values stored in a one-dimensional C buffer?

The C++ contract will store matrices in row-major order:

    r11, r12, r13, r21, r22, r23, r31, r32, r33

Fortran reshape fills a matrix using column-major ordering. The ADI binding
provides TransposeDCM so callers that pass the row-major sequence can request
the corresponding transpose after reshape. Therefore this adapter will set
TransposeDCM to 1 internally.

TransposeDCM must not be a YAML option. Letting users toggle it would make a
wrong coordinate convention look like a configurable case property.

The contract test must use a non-symmetric rotation matrix, preferably a
combined rotation about at least two axes. An identity matrix cannot reveal a
transpose error. A second test must establish whether the Chrono rotation is
local-to-global or global-to-local and perform the physical inversion in the
marshaller if ADI expects the opposite convention.

Chrono currently publishes Cardan XYZ angles for rigid reference frames. The
AeroDyn participant may convert those angles to its internal rotation type, but
the conversion order, sign, units, and active/passive interpretation must be
covered by the same contract test.

### 4.4 Turbine origin and relative positions

The ADI binding treats TurbOrigin_C as a fixed global turbine reference and adds
it to the hub, nacelle, blade-root, and blade-node positions passed to the
binding. Motion positions must therefore be relative to that origin:

    p_ADI = p_Chrono_global - p_turbine_origin_global

No additional Chrono input parser is required. Chrono already knows the
turbine origin, and the origin is included as a role on the Chrono-owned
reference-frame mesh.

For version 1:

- the turbine origin is fixed;
- its axes are aligned with the Chrono global axes;
- position subtraction is applied during setup and every motion update;
- global linear and angular velocities are not changed by a pure translation;
- orientations are not changed by a pure translation;
- returned forces and moments remain in the agreed global basis.

A rotated or moving turbine-origin frame requires full vector and DCM
transforms and is a later extension.

### 4.5 Rigid-body points are not a conceptual blocker

A point fixed to a rigid body inherits the body motion. If a blade coupling
station also has a defined local section frame, let its fixed transform in the
body be:

    position offset: r_BN
    orientation offset: R_BN

At runtime:

    p_GN = p_GB + R_GB r_BN
    R_GN = R_GB R_BN
    v_GN = v_GB + omega_GB × (R_GB r_BN)
    omega_GN = omega_GB

Chrono already provides the position and linear velocity of a rigid-body point.
The missing part in the inspected adapter is wiring, not rigid-body mechanics:
RIGID_BODY_POINTS currently does not publish rotations or angular velocities.

For the rigid milestone, extend a coupling point from a bare position to an
optional body-fixed frame. Preserve the existing position-only API by treating
its local orientation as identity. Then allow RIGID_BODY_POINTS to write
rotations and angular velocities for frame-aware points.

This keeps all fixed section orientations, including built-in blade twist, in
the Chrono model. The AeroDyn YAML does not duplicate them. If the Chrono
extension cannot be made immediately, reconstructing node frames in the
AeroDyn participant from a blade reference frame is an acceptable temporary
fallback, but it must receive the fixed local section frames from Chrono as
initial data.

Returned force is applied at the body point. Returned moment is applied to the
owning body. The sign and reference point must be checked with a one-node
wrench test.

### 4.6 Flexible blades and ChNodeFEAxyzrot

ChNodeFEAxyzrot is the preferred first flexible-node type because every node
has explicit translational and rotational degrees of freedom. It directly
provides:

- nodal position;
- nodal quaternion or rotation;
- linear velocity;
- angular velocity;
- a natural location for nodal force;
- a natural location for nodal moment.

This is a close match to the ADI motion and load interface. It is also suitable
for blades with torsional deformation, provided that the selected beam element
and section model include the required torsional stiffness and coupling terms.
The node type alone does not define blade torsional behavior.

Chrono includes rotation-based beam families compatible with ChNodeFEAxyzrot.
A tapered Timoshenko formulation is a strong candidate for wind-turbine blades,
but the final element and section choice should be verified against the
required shear, torsion, taper, and bend–twist coupling behavior.

The future FEA provider will populate exactly the same BladeNodeKinematics
objects used by the rigid provider. AdiMarshaller and AeroDynDriver will remain
unchanged.

### 4.7 Why ANCF is different

The additional work required for ANCF is not caused by AeroDyn using positions
relative to the turbine origin. Relative-position conversion is the same for
any structural formulation.

The difference is the structural state representation:

- ChNodeFEAxyzrot stores an explicit nodal rotation and angular velocity.
- ANCF nodes store position plus slope or director gradients.
- An ANCF section orientation must be reconstructed from those gradients.
- Angular velocity must be derived consistently from the time derivatives of
  the gradients.
- A returned aerodynamic moment may need to be distributed into generalized
  forces associated with the gradient coordinates rather than applied as a
  direct nodal torque.

Some fully parameterized ANCF beam formulations can model torsion. A
single-slope cable-style ANCF formulation is not sufficient for general blade
twist. ANCF therefore needs a separate Chrono-side kinematics-and-load provider,
not a change to the ADI marshaller.

---

## 5. Coupling contract

### 5.1 Mesh organization

Version 1 should use, per turbine:

1. one reference-frame mesh;
2. one blade coupling mesh per blade.

The reference-frame mesh has a fixed semantic order:

1. turbine origin;
2. hub;
3. nacelle;
4. blade roots in the same order as the blade list in the coupling YAML.

One blade mesh per blade avoids a user-supplied flat blade-index array. The
AeroDyn participant concatenates blade meshes in YAML order and generates
MeshPtToBladeNum_C internally using one-based blade indices.

This layout requires mesh membership to be explicit on the Chrono side. Each
interface must select the bodies, body-fixed frames, points, or FEA nodes that
belong to that mesh and preserve their configured order. A single global list
of coupling bodies reused by every interface is not sufficient.

Each received vertex ID is kept with its typed node. Loads are written back
using the same vertex IDs, so no coordinate-based lookup is needed after
initialization.

For the first implementation, participants are single-rank and mesh topology is
constant. A parallel partitioning and semantic-role strategy can be added
later.

### 5.2 Data fields

| Mesh | Chrono writes | AeroDyn writes |
|---|---|---|
| Reference-frame mesh | Positions, Rotations, LinearVelocities, AngularVelocities | none |
| Each blade mesh | Positions, Rotations, LinearVelocities, AngularVelocities | Forces, Torques |

All motion and load fields are three-dimensional vectors. Rotations in the
current Chrono adapter are Cardan XYZ angle vectors in radians. This is a
transport representation only; the AeroDyn domain model should convert them
immediately to a strongly named orientation type.

Version 1 requires PointLoadOutput = 1, so ADI_C_GetRotorLoads returns force in
newtons and moment in newton-metres at each structural coupling point.

### 5.3 Rotor type

The public configuration uses:

    rotor_type: VAWT

or:

    rotor_type: HAWT

The parser converts this enum to TurbineIsHAWT_C only at the ADI boundary. A
Boolean should not appear in the domain model.

The currently inspected ADI implementation maps HAWT to AeroProjMod 1 and VAWT
to AeroProjMod 3. AeroProjMod 3 is currently associated with the lifting-line
path used for OLAF with VAWTs. This is an upstream limitation to document and
test; it is not a reason to encode HAWT assumptions in the adapter.

### 5.4 Environmental values

The C binding requires environmental defaults before AeroDyn.dat is parsed.
The coupling YAML therefore owns those ADI pre-initialization values.

Use an enum rather than a Boolean:

| YAML mode | ADI MHK value | Required density |
|---|---:|---|
| wind | 0 | air_density |
| mhk_fixed_bottom | 1 | water_density |
| mhk_floating | 2 | water_density |

The parser passes the selected density as the ADI default fluid density.
Overlapping fields in module inputs should use OpenFAST DEFAULT semantics where
supported. If a module input explicitly repeats a value, it must either be
validated against the manifest or reported as an unverifiable duplicate; it
must not silently disagree.

### 5.5 Time contract

ADI_C_Init requires a constant time step. Version 1 therefore uses:

- one fixed ADI time step;
- a preCICE time window equal to that time step;
- explicit coupling;
- no adaptive reduction of ADI dt;
- no silent use of min(preCICE dt, ADI dt).

At startup, reject a configuration in which the preCICE maximum step is not
compatible with the ADI step. Exact subcycling may be added later, but every
ADI substep must still use the fixed initialization time step.

Empty checkpoint methods are valid only with an explicit preCICE scheme. The
adapter must detect an implicit scheme requesting rollback and fail with a
clear message.

---

## 6. Coupling YAML design

### 6.1 Purpose

The file is an adapter manifest, not a new AeroDyn input format. A suggested
name is:

    aerodyn_precice.yaml

It contains only information needed to construct the preCICE participant and
the ADI calls that cannot be obtained from AeroDyn.dat, InflowWind.dat, or the
Chrono-owned coupling meshes.

### 6.2 Version 1 example

    schema_version: 1

    participant:
      name: AeroDyn
      precice_config: ./precice-config.xml

    modules:
      aerodyn_file: ./AeroDyn.dat
      inflowwind_file: ./InflowWind.dat

    simulation:
      time_step: 0.01
      end_time: 180.0
      interpolation_order: 1

    environment:
      mode: wind
      gravity: 9.80665
      air_density: 1.225
      kinematic_viscosity: 1.464e-5
      speed_of_sound: 335.0
      atmospheric_pressure: 101325.0
      vapor_pressure: 1700.0
      water_depth: 0.0
      msl_to_swl: 0.0

    coupling:
      data:
        positions: Positions
        rotations: Rotations
        linear_velocities: LinearVelocities
        angular_velocities: AngularVelocities
        forces: Forces
        torques: Torques

    turbines:
      - id: turbine-1
        rotor_type: VAWT
        reference_frames_mesh: Turbine-1-ReferenceFrames
        blades:
          - id: blade-1
            mesh: Turbine-1-Blade-1
          - id: blade-2
            mesh: Turbine-1-Blade-2
          - id: blade-3
            mesh: Turbine-1-Blade-3

    outputs:
      root_name: output/turbine
      adi_file_format: ascii
      output_time_step: 0.01
      write_vtk: none
      debug_level: 0

There are deliberately no structural positions, DCMs, node counts, blade
indices, or flat arrays in this file.

### 6.3 Parser rules

The parser should:

- require schema_version;
- reject unsupported versions;
- reject unknown keys by default;
- report the complete YAML key path and source location for an error;
- resolve relative paths from the manifest directory, not the process working
  directory;
- require scalar paths for aerodyn_file and inflowwind_file;
- reject inline mappings or sequences for those fields in version 1;
- require files to exist and be readable before creating the ADI participant;
- parse enums case-insensitively but store a canonical enum value;
- validate finite numeric values and documented ranges;
- require interpolation_order to be 1 or 2;
- require a positive fixed time_step and end_time;
- require at least one turbine and one blade per turbine;
- require unique turbine IDs, blade IDs, and mesh names;
- reject mixed HAWT and VAWT turbines in one ADI instance while the linked
  binding requires one common AeroProjMod;
- derive num_blades from the blade list;
- derive point counts from received meshes;
- derive MeshPtToBladeNum_C from blade-list order;
- parse ADI output format as none, ascii, binary, or both rather than a
  Boolean;
- validate that every required preCICE mesh and data field exists;
- produce an immutable validated configuration object.

Parsing and validation should happen before ADI_C_PreInit. A partially
initialized ADI library should never be used to discover basic YAML mistakes.

### 6.4 Mapping to the ADI API

| YAML or received value | ADI destination |
|---|---|
| number of turbines | ADI_C_PreInit NumTurbines_C |
| environment block | ADI_C_PreInit environmental arguments and MHK |
| rotor_type | ADI_C_SetupRotor TurbineIsHAWT_C |
| Chrono origin vertex | ADI_C_SetupRotor TurbOrigin_C |
| Chrono initial reference/node frames | ADI_C_SetupRotor initial geometry |
| generated blade mapping | ADI_C_SetupRotor MeshPtToBladeNum_C |
| module paths | ADI_C_Init file-path mode |
| interpolation and time settings | ADI_C_Init |
| current Chrono motion | ADI_C_SetRotorMotion |
| ADI nodal wrench | preCICE Forces and Torques |

---

## 7. Internal code organization

The current baseline has the right broad separation between the preCICE
participant and the raw binding, but TurbineGeometry and raw arrays should be
moved out of the ABI header.

Recommended organization:

| File or component | Responsibility |
|---|---|
| AeroDynConfig.h/.cpp | Typed configuration, YAML parsing, defaults, validation, path resolution |
| AeroDynTypes.h | RotorType, EnvironmentMode, Orientation, ReferenceFrameKinematics, BladeNodeKinematics, BladeNodeLoad |
| AeroDynCBinding.h | Raw extern C declarations and authoritative ABI constants only |
| AdiMarshaller.h/.cpp | Origin transform, orientation conversion, flattening, one-based indexing, buffer validation |
| AeroDynDriver.h/.cpp | RAII ownership of ADI, global/per-turbine call ordering, output buffers, error translation |
| AeroDynPreciceAdapter.h/.cpp | Received mesh discovery, preCICE data exchange, typed state construction, coupling lifecycle |
| OfPreciceAdapter.h/.cpp | Generic preCICE lifecycle, including pre- and post-initialize hooks and received-mesh support |
| main.cpp | CLI parsing, construction, run, and top-level error reporting |
| tests/ | Parser, math, marshalling, driver-sequence, and coupled integration tests |

### 7.1 Dependency direction

Dependencies should point inward:

1. AeroDynTypes has no preCICE, Chrono, YAML, or ADI dependency.
2. AeroDynConfig creates typed values.
3. Chrono/preCICE providers create typed kinematics and consume typed loads.
4. AdiMarshaller translates typed values to raw buffers.
5. AeroDynDriver alone calls ADI_C_*.
6. main.cpp wires the layers together.

This keeps the code succinct and makes the coordinate and ABI rules testable
without starting Chrono, preCICE, or AeroDyn.

### 7.2 ADI error handling and lifetime

AeroDynDriver should:

- call ADI_C_End exactly once after successful pre-initialization;
- use best-effort cleanup during exception unwinding;
- preserve information and warning diagnostics instead of dropping them;
- throw for fatal or abort-level errors with the ADI call and turbine ID;
- centralize string buffers and their authoritative sizes;
- allocate output-channel values using NumChannels_C returned by ADI_C_Init;
- parse channel names and units using the exact upstream fixed-width contract;
- reject a second initialization on the same live driver;
- make copy construction and copy assignment unavailable.

Do not hardcode a 100-element output array. Do not duplicate error-buffer
lengths in several source files.

---

## 8. Solver lifecycle and ADI call sequence

### 8.1 Initialization

1. Parse and validate aerodyn_precice.yaml.
2. Construct the preCICE participant.
3. Declare received meshes and data.
4. Set direct-access regions if required.
5. Initialize preCICE.
6. Retrieve received mesh vertex IDs and coordinates.
7. Read initial reference and blade kinematics.
8. Validate counts, roles, transforms, DCMs, and data dimensions.
9. Build typed rotor states.
10. Marshal initialization buffers.
11. Call ADI_C_PreInit once.
12. Call ADI_C_SetupRotor once per turbine.
13. Call ADI_C_Init once.
14. Store NumChannels_C and allocate exact output buffers.

### 8.2 One coupling step

For each fixed time step:

1. read current reference and blade motion from preCICE;
2. convert Cardan angles to the internal orientation representation;
3. subtract the turbine origin from all ADI position inputs;
4. marshal all turbine motion buffers;
5. call ADI_C_SetRotorMotion once per turbine;
6. call ADI_C_UpdateStates once for the global ADI instance;
7. call ADI_C_CalcOutput once;
8. call ADI_C_GetRotorLoads once per turbine;
9. optionally call ADI_C_GetDiskAvgVel once per turbine;
10. unmarshal force and moment for each blade mesh;
11. write Forces and Torques through preCICE;
12. advance preCICE by the fixed step.

The order above matches the ADI wrapper usage pattern. The driver layer should
own it so that no preCICE callback can accidentally reorder global and
per-turbine calls.

### 8.3 Shutdown

1. call ADI_C_End once;
2. finalize preCICE;
3. flush adapter-owned output and diagnostics;
4. preserve the first fatal exception if cleanup also reports an error.

---

## 9. Phased implementation plan

### Phase 0 — Freeze and test the interface contracts

Goal: remove ambiguity before adding YAML parsing.

Project work:

- Pin the OpenFAST, Chrono feature-branch, and preCICE versions used for the
  first integration.
- Compare AeroDynCBinding.h against the exact linked ADI library revision.
- Centralize integer widths, string lengths, output-channel widths, and error
  severity rules.
- Confirm that the C caller uses the turbine indexing required by the linked
  Fortran binding and that blade numbers are one-based.
- Set PointLoadOutput to 1.
- Set TransposeDCM to 1 for the row-major C++ contract.
- Replace the MHK Boolean with an enum mapped to 0, 1, or 2.
- Replace ADI output and VTK mode Booleans/integers in public configuration
  with named enums and convert them only at the C-binding boundary.
- Add the turbine-origin subtraction.
- Allocate output values from NumChannels_C.
- Replace the current hard-coded output-channel value count.
- Make zero acceleration an explicit version 1 assumption.
- Add non-symmetric DCM, known-axis, origin-offset, index, and buffer-size
  tests.
- Add a small standalone ADI smoke driver independent of preCICE.

Upstream facts to record, not implement in this phase:

- the status of the OpenFAST YAML-input branch;
- the current VAWT/HAWT AeroProjMod selection;
- the current absence of wired Chrono FEA mesh paths;
- any ADI limitations discovered by the smoke case.

Exit criterion: a direct ADI run initializes, steps, returns loads, and passes
the coordinate and buffer contract tests.

### Phase 1 — Typed configuration and paths-only YAML parser

Goal: replace hard-coded AerodynGlobalSettings and caller-built
TurbineGeometry.

Tasks:

- Add AeroDynTypes.
- Add AeroDynConfig and a yaml-cpp dependency declared directly in CMake.
- Implement schema version 1.
- Implement strict diagnostics and path resolution.
- Parse module paths, time, environment, participant, data names, outputs,
  turbine IDs, rotor types, and mesh bindings.
- Reject structural coordinates and inline module mappings.
- Derive blade counts and validate uniqueness.
- Add valid, defaulted, and invalid parser fixtures.
- Update main.cpp to accept the manifest path.

Exit criterion: the standalone ADI smoke driver can be launched entirely from
the coupling YAML and normal AeroDyn/InflowWind files, except for geometry,
which will arrive through preCICE in Phase 2.

### Phase 2 — Single-source geometry and rigid-point frame support

Goal: obtain all structural initialization data from Chrono without a second
geometry definition.

Tasks:

- Refactor OfPreciceAdapter into pre-initialize and post-initialize hooks.
- Support meshes received with direct API access.
- Retrieve vertex IDs and coordinates after preCICE initialize.
- Define and enforce the reference-frame vertex contract.
- Use one blade mesh per blade.
- Add per-interface Chrono membership and ordering for body references,
  body-fixed frames, and points.
- Extend Chrono rigid coupling points to optionally carry a body-fixed local
  frame.
- Update Chrono data validation and read/write dispatch so frame-aware
  RIGID_BODY_POINTS may publish rotations and angular velocities.
- Publish rigid-point rotations and angular velocities.
- Keep the existing position-only rigid-point API compatible.
- Build typed rotor and blade states from the received data.
- Add startup validation for counts, dimensions, DCM orthonormality,
  determinant, origin, and finite values.

Exit criterion: a zero-load preCICE exchange reproduces Chrono rigid-body point
positions, frames, and velocities in the AeroDyn participant with no structural
coordinates in the AeroDyn coupling YAML.

### Phase 3 — Rigid VAWT end-to-end coupling

Goal: deliver the first supported co-simulation.

Tasks:

- Configure one rigid VAWT in Chrono.
- Pass origin, hub, nacelle, root, and blade-station kinematics.
- Initialize ADI from AeroDyn.dat and InflowWind.dat paths.
- Return point forces and moments per station.
- Apply each force at its body point and each moment to the owning body.
- Run serial explicit coupling with one fixed time step.
- Add zero-wind, steady-wind, and prescribed-rotation tests.
- Compare against the standalone ADI driver at matching motions.

Exit criterion: the rigid VAWT runs to the requested end time, load histories
match the reference within agreed engineering tolerances, and shutdown is
clean.

### Phase 4 — HAWT compatibility

Goal: prove that VAWT-first design choices did not introduce axis-specific
assumptions.

Tasks:

- Add one rigid HAWT smoke case.
- Verify rotor_type conversion and reference-frame roles.
- Reuse the same YAML schema, mesh layout, domain types, and marshaller.
- Add an orientation case with a nontrivial yaw and blade azimuth.
- Compare against a direct ADI HAWT case.

Exit criterion: the HAWT case requires data changes only, not a separate code
path beyond the rotor-type enum.

### Phase 5 — Flexible blades with ChNodeFEAxyzrot

Goal: replace the rigid kinematics provider without changing the AeroDyn side
contract.

Chrono-side tasks:

- Implement FEA mesh loading and registration for FEA_MESH_NODES.
- Select and validate ChNodeFEAxyzrot nodes from a ChMesh.
- Extend Chrono coupling-data validation so FEA nodes may write rotations and
  angular velocities and may receive torques.
- Write nodal positions, rotations, linear velocities, and angular velocities.
- Read nodal forces and torques.
- Apply those loads consistently to the node generalized coordinates.
- Add checkpoint support for the Chrono structural state if implicit coupling
  is investigated later.

Modeling tasks:

- Select a torsion-capable ChNodeFEAxyzrot beam element and section model.
- Represent spanwise mass, bending stiffness, torsional stiffness, offsets,
  and any bend–twist coupling required by the blade.
- Define how aerodynamic coupling stations correspond to structural nodes or
  create a dedicated interpolation layer.

AeroDyn-side expectation:

- no YAML schema change other than optional provider metadata;
- no AdiMarshaller change;
- no AeroDynDriver call-sequence change;
- the same BladeNodeKinematics and BladeNodeLoad objects.

Exit criterion: a flexible blade shows bending and torsional response under
aerodynamic loading, and the rigid limit converges to the Phase 3 result.

### Phase 6 — Hardening and multiple turbines

Goal: move from a research demonstrator to a maintainable adapter.

Tasks:

- Preserve the global ADI call pattern: PreInit once, SetupRotor per turbine,
  Init once, SetRotorMotion per turbine, global state/output calls once, then
  load retrieval per turbine.
- Add a two-turbine initialization and indexing test.
- Add structured logging with turbine, blade, mesh, vertex, and ADI call
  context.
- Add sanitizer and leak-test builds.
- Add version reporting for OpenFAST, Chrono, preCICE, compiler, and schema.
- Add CI fixtures that do not require the full production turbine model.
- Write user documentation and a minimal runnable example.

Exit criterion: repeatable builds, deterministic configuration errors, no
known buffer overruns or lifetime errors, and automated rigid VAWT plus HAWT
smoke coverage.

---

## 10. Validation strategy

### 10.1 Unit tests

- YAML defaults, path resolution, enum parsing, unknown keys, and diagnostics.
- Environment-mode and density selection.
- Cardan XYZ conversion with known rotations.
- DCM storage order and TransposeDCM behavior.
- DCM orthonormality and determinant rejection.
- Turbine-origin subtraction.
- Rigid-frame inheritance and omega cross r point velocity.
- Float/double conversion and finite-value rejection.
- One-based turbine and blade indexing.
- Concatenation and splitting of one mesh per blade.
- Exact buffer sizes for zero, one, and many nodes.

### 10.2 Component tests

- Fake ADI implementation recording call order and buffer contents.
- AeroDynDriver initialization failure and RAII cleanup.
- Information, warning, severe, and fatal ADI diagnostics.
- Exact NumChannels_C allocation.
- Received-mesh vertex ID preservation.
- Fixed point count over the complete run.
- Point force and moment round trip.

### 10.3 Coupled tests

1. Static rigid blade, zero wind: near-zero aerodynamic load as appropriate
   for the case.
2. Static rigid blade, uniform wind: stable, repeatable load.
3. Prescribed rigid rotation: Chrono-to-AeroDyn kinematics compared point by
   point.
4. Rigid VAWT: comparison with the standalone ADI driver using identical
   motion histories.
5. Rigid HAWT: orientation and rotor-type regression.
6. Flexible cantilever without AeroDyn: verify structural torsion and bending
   independently before coupling.
7. Flexible coupled blade: load and response convergence with time-step and
   mesh refinement.

Comparison should use documented absolute and relative tolerances. Bit-for-bit
agreement is not required across compilers and library builds unless a specific
test demonstrates that it is stable.

---

## 11. Version 1 definition of done

Version 1 is complete when:

- a user launches the AeroDyn participant from one coupling YAML file;
- AeroDyn.dat and InflowWind.dat are supplied as paths;
- no structural position or orientation is duplicated in that YAML;
- Chrono owns all coupling meshes;
- one rigid VAWT runs through preCICE with a fixed explicit step;
- the same executable and schema run a rigid HAWT smoke case;
- DCM and turbine-origin contracts are covered by automated tests;
- forces and moments are applied at the correct Chrono points;
- ADI output buffers use the returned channel count;
- diagnostics identify the turbine, blade, mesh, and failing ADI call;
- ADI_C_End and preCICE finalization occur exactly once;
- results match a direct ADI reference within agreed tolerances;
- the flexible-node provider can be added without changing AeroDynDriver or
  AdiMarshaller.

---

## 12. Risks and controls

| Risk | Control |
|---|---|
| Silent orientation transpose or frame inversion | Non-symmetric DCM and known-axis contract tests |
| Double-defined geometry drifts between solvers | Chrono-owned received meshes; no structural coordinates in coupling YAML |
| VAWT upstream projection limitations | Pin OpenFAST revision, document AeroProjMod behavior, keep a VAWT regression case |
| FEA enum mistaken for completed support | Treat FEA as Phase 5 and test every registration/read/write path |
| Fixed ADI dt conflicts with preCICE | Validate equal version 1 time windows and reject incompatible configurations |
| Rigid point has no defined section orientation | Use frame-aware body-fixed coupling stations |
| Every Chrono interface accidentally reuses one global body list | Make mesh membership and ordering explicit per interface |
| Euler-angle singularity | Convert immediately to a typed orientation and consider quaternion/DCM transport later |
| Wrong load reference point or sign | One-node force-and-moment balance test |
| ADI ABI changes | Pin revisions, centralize constants, and run ABI smoke tests |
| Implicit scheme used with no rollback | Explicit-scheme guard and startup failure |

---

## 13. Potential expansion points

- Support exact ADI subcycling inside larger preCICE time windows.
- Investigate whether ADI correction-step behavior can satisfy preCICE
  implicit iterations; otherwise add or request explicit ADI state
  checkpoint/restore support.
- Support multiple turbines and mixed structural models in one ADI participant.
  Note that the inspected ADI implementation requires turbines in one instance
  to use the same AeroProjMod.
- Support moving or rotated turbine-origin frames with complete coordinate and
  load transformations.
- Transport quaternions or DCMs directly through scalar data fields to avoid
  Cardan-angle singularities.
- Add optional structural-to-coupling interpolation when aerodynamic coupling
  stations do not coincide with FEA nodes.
- Add distributed line-load output as an alternative to point loads.
- Add translational and rotational acceleration exchange if a validated case
  requires it.
- Add multi-rank direct mesh access and an explicit semantic-role mechanism
  that does not depend on serial vertex order.
- Consume upstream AeroDyn/InflowWind YAML module files when that interface is
  stable, while retaining the paths-only adapter manifest.
- Develop HydroDyn or other OpenFAST module participants under separate,
  module-specific plans.
- Add ANCF blade providers. Fully parameterized ANCF beams may support torsion,
  but each provider must reconstruct an orthonormal section frame and angular
  velocity from gradient coordinates and must map returned aerodynamic moments
  into the correct ANCF generalized forces. This remains a Chrono-side provider
  extension; the AeroDyn marshalling and driver layers should remain unchanged.
