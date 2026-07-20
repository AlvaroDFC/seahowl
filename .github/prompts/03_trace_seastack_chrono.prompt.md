Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc)

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source)

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/mkdocs.yml#1-1), lines 1 to 80

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source/dev-guide)

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/mkdocs.yml#80-80), lines 80 to 120

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source/dev-guide/architecture)

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source/dev-guide/core-workflow.md#1-1), lines 1 to 100

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source/dev-guide/architecture/seahowl_openfast_interface.md#1-1), lines 1 to 80

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source/dev-guide/sim-workflow.md)

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source/dev-guide/architecture/diagrams)

Read [](file:///c%3A/Users/adiazfl/Projects/Seahawl/code/seahowl/doc/source/dev-guide/architecture/seahowl_openfast_interface.md#1-1), lines 1 to 40

Created memory file [](file:///c%3A/Users/adiazfl/AppData/Roaming/Code/User/workspaceStorage/e7fc4a02e4e96e32559c7761a7b0a382/GitHub.copilot-chat/memory-tool/memories/NDEyY2MyYmEtYjlkNS00MGExLTg4NTktOTZiYzFkYjk4MDdj/plan.md)

## Plan: SEAHOWL ↔ Chrono Interface Documentation

**TL;DR:** Create one new page `seahowl_chrono_interface.md` in the existing architecture section of the dev-guide, mirroring the style of the OpenFAST interface page. Register it in mkdocs.yml.

---

### Files to touch

- `seahowl/doc/source/dev-guide/architecture/seahowl_chrono_interface.md` — CREATE
- mkdocs.yml — add nav entry under `Dev Guide > architecture`

---

### Page structure

**1. Overview**
Short paragraph: Seahowl owns the hierarchical semantic model; Chrono owns the physics solver. `ChAssembly` is Chrono-internal and is never touched by Seahowl directly. The only Seahowl object that holds a raw Chrono handle is `SystemElastoChrono` (as `chobj: shared_ptr<chrono::ChSystem>`).

---

**2. Class hierarchies**

Three parallel trees shown as a Mermaid class diagram:

*Tree 1 — Component hierarchy (structural semantics)*
```
ComponentElasto (abstract)
  ├── ComponentElastoFEA    owns nodes[], elements[]
  │     ├── TowerElasto
  │     ├── BladeElasto
  │     └── MooringElastoFEA
  ├── RotorElasto           owns BodyElasto (hub) + vector<BladeElasto>
  ├── TurbineElasto         owns TowerElasto, RotorNacelleAssemblyElasto, FoundationElasto
  └── SystemElasto (abstract: assemble, step, do_statics)
        └── SystemElastoChrono  ──► chrono::ChSystem
```

*Tree 2 — Entity hierarchy (individual physics DOFs)*
```
EntityDynamic → EntityLoadable
  ├── BodyElasto (abstract)  →  BodyElastoChrono  ──► chrono::ChBody
  └── NodeElasto (abstract)  →  NodeElastoChrono  ──► chrono::ChNodeFEAxyzrot
ElementElasto (abstract)     →  (Chrono variants) ──► chrono::ChElementBeam*
MeshElasto (abstract)        →  MeshElastoChrono  ──► chrono::fea::ChMesh
```

---

**3. Object combination table**

| Seahowl class | Holds | Chrono class |
|---|---|---|
| `SystemElastoChrono` | `shared_ptr<chrono::ChSystem> chobj` | `chrono::ChSystem` |
| `BodyElastoChrono` | `shared_ptr<chrono::ChBody> chobj` | `chrono::ChBody` |
| `NodeElastoChrono` | `shared_ptr<chrono::fea::ChNodeFEAxyzrot> chobj` | `chrono::fea::ChNodeFEAxyzrot` |
| `MeshElastoChrono` | `shared_ptr<chrono::fea::ChMesh> chobj` | `chrono::fea::ChMesh` |
| `LinkChronoBase` | `shared_ptr<chrono::ChLinkBase> chobj` | various `ChLink*` subclasses |

---

**4. Assembly: one-time registration**

A Mermaid flowchart of the `assemble()` cascade, showing how `SystemElastoChrono::add()` unwraps each abstract entity and calls `chobj->Add()` on `ChSystem`, ending up in `ChAssembly` as a flat list.

---

**5. Per-timestep communication**

A Mermaid sequence diagram covering one cycle for a `Blade`:

```
core::Blade::prestep  → NodeElastoChrono::accumulate_force → ChLoadForceTorque (write)
SystemElastoChrono::step → ChSystem::DoStepDynamics (Chrono integrates)
core::Blade::poststep → NodeElastoChrono::get_position → ChNodeFEAxyzrot::GetPos (read)
```

Accompanied by a table of the three communication moments (prestep/step/poststep).

---

**6. HydroChrono special case**

One paragraph noting that `FloaterHydroChrono` inherits `FloaterElasto` rather than a fluid class — it is the exception where hydrodynamic loads are computed inside Chrono itself, crossing the usual elasto/fluid boundary.

---

### Verification
1. Run run_mkdocs.bat (or `mkdocs build`) — must complete with no errors.
2. New page visible under **Dev Guide > Architecture** in the rendered site.
3. All three Mermaid diagrams render without broken nodes.

---

Does this structure cover what you had in mind, or would you like to adjust the scope (e.g. add a section on coordinate system conversions between IEC and Chrono conventions, which also lives in chrono_adapters.cpp)?