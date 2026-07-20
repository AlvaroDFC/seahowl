# OpenFAST Module Matrix

| OpenFAST module | SEAHOWL adapter class | pImpl / internal struct | C-binding externs (Init → Update → CalcOutput → End) | Build guard | JSON config key |
|---|---|---|---|---|---|
| **AeroDyn + InflowWind (ADI)** | `AeroDynAdapter` owned by `TurbineAeroDyn` | `AeroDynInflowLib` | `ADI_C_PreInit` · `ADI_C_SetupRotor` · `ADI_C_Init` → `ADI_C_SetRotorMotion` · `ADI_C_UpdateStates` → `ADI_C_CalcOutput` · `ADI_C_GetRotorLoads` · `ADI_C_GetDiskAvgVel` → `ADI_C_End` | `HAVE_AERODYN` | `aero.solver = "aerodyn"` |
| **InflowWind (standalone)** | `InflowWindAdapter : WindModel` | `InflowWindLib` | `IfW_C_Init` → *(no update)* → `IfW_C_CalcOutput` → `IfW_C_End` | `HAVE_INFLOWWIND` | `wind.type = "inflowwind"` |
| **HydroDyn** | `HydroDynAdapter` owned by `FloaterHydroDyn` or `MonopileHydroDyn` | `HydroDynLib` | `HydroDyn_C_Init` → `HydroDyn_C_UpdateStates` → `HydroDyn_C_CalcOutput_and_AddedMass` → `HydroDyn_C_End` | `HAVE_HYDRODYN` | `foundation.options.solver_hydro = "hydrodyn"` (monopile) or `floater.type = "hydrodyn"` |
| **SeaState** | `SeaStateAdapter : WaveModel` | `SeaStateLib` | `SeaSt_C_PreInit` · `SeaSt_C_Init` → *(no update)* → `SeaSt_C_GetFluidVelAcc` · `SeaSt_C_GetSurfElev` · `SeaSt_C_GetDens` → `SeaSt_C_End` | `HAVE_SEASTATE` | `sea.type = "seastate"` |
| **DISCON** | `ControllerDISCON` via `DisconInterface` | avrSWAP float array (dlopen) | `DisconInterface::Init` (dlopen) → `DisconInterface::Call` (each step) | *(runtime)* | `controller.type = "discon"` |
| **MoorDyn** | — | — | — | — | *(not integrated)* |

## SEAHOWL → OpenFAST inputs per module

| OpenFAST module | Data sent by SEAHOWL |
|---|---|
| AeroDyn + InflowWind | Hub position/orientation/velocity/acceleration · Nacelle position/orientation/velocity/acceleration · Blade-root position/orientation/velocity/acceleration (×N blades) · Blade mesh node position/orientation/velocity/acceleration (×N blades × M nodes) · InflowWind input file path (resolved at `setup_environment`) |
| InflowWind (standalone) | Query point position (x, y, z) · time |
| HydroDyn | Platform/node position (x, y, z, roll, pitch, yaw) · node translational and rotational velocity · node translational and rotational acceleration · SeaState input file path |
| SeaState | Query point position (x, y, z) · time |
| DISCON | Rotor speed · Generator speed · Rotor azimuth · Blade pitch angles (×N blades) · Blade root flapwise/edgewise moments (×N blades) · Tower-top translational acceleration (fore-aft, side-side) · Nacelle rotational acceleration (roll, pitch, yaw in shaft frame) · Hub-height wind speed · Yaw error · Generated power · Shaft power · Time · Time step |

## OpenFAST → SEAHOWL outputs per module

| OpenFAST module | Data returned to SEAHOWL |
|---|---|
| AeroDyn + InflowWind | Distributed blade forces + moments at each mesh node (`MeshFrc[6×NumMeshPts]`) · Disk-averaged wind velocity (3-component) · Output channel values |
| InflowWind (standalone) | Wind velocity vector (3-component) at the queried point |
| HydroDyn | Node loads (`NodeFrc[6×NumNodePts]`) · Added-mass matrix (`NodeAdm[6N × 6N]`) · Output channel values |
| SeaState | Fluid velocity (3-component) · Fluid acceleration (3-component) · Node-in-water flag · Wave surface elevation · Water density |
| DISCON | Collective pitch demand · Individual blade pitch demands (×3) · Generator electrical torque demand · Yaw rate demand |
