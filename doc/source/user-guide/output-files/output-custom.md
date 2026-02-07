# Custom CSV Output files

In addition to the default turbine output, you can create custom CSV files to record any quantity accessible through the SEAHOWL API. Custom CSVs are created via the `OutputManager.create_new_csv()` method and populated with user-defined functions using `CustomCSV.add_function()`.

Custom CSV files are written to the same output folder as the default outputs, and follow the same output time interval (`dt_output`).

## Supported Return Types

The `add_function()` method accepts functions that return:

| Return type | Columns generated |
|-------------|-------------------|
| scalar (`double`) | 1 column |
| 3D vector (`Vector3d`) | 3 columns with `x`, `y`, `z` suffixes |
| quaternion (`Quaternion`) | 4 columns with `w`, `x`, `y`, `z` suffixes |
| list of doubles (`std::vector<double>`) | N columns with `dim0`, `dim1`, ... suffixes |

## Python Usage

Custom CSVs are especially convenient from Python using lambda functions. Create the CSV and add functions **before** calling `simulation.initialize()`.

```python
import seahowl

simulation = seahowl.core.Simulation()
simulation.dt = 0.025
simulation.duration = 200.0

# output options
simulation.outputs.dt_output = 0.0  # 0 = output all time steps
simulation.outputs.has_csv = True
simulation.outputs.set_output_folder("./output")

# load turbine and environment
system_core = simulation.system_core
seahowl.io.add_turbine_to_system_from_file("turbine.json", system_core)
turbine = system_core.turbines[0]
system_core.env_model = seahowl.io.get_environmental_model_from_file("env.json")

# create a custom CSV
mycsv = simulation.outputs.create_new_csv("my_outputs.csv")

# add columns with lambda functions
mycsv.add_function("time [s]", lambda: system_core.get_time())
mycsv.add_function(
    "blade1 tip position [m]",
    lambda: turbine.elasto.rna.rotor.blades[0].nodes[-1].get_position(),
)

# initialize and run
simulation.initialize()

while system_core.get_time() < simulation.duration:
    simulation.step()
```

The resulting `output/my_outputs.csv` file will contain:

```
time [s],blade1 tip position x [m],blade1 tip position y [m],blade1 tip position z [m],
0.000000,1.234,5.678,150.123,
0.025000,1.235,5.679,150.124,
...
```

## C++ Usage

```cpp
#include <seahowl/io/write_csv.h>

// during setup, before initialization
auto& mycsv = simulation.get_outputs().create_new_csv("my_outputs.csv");
mycsv.add_function("time [s]", [&]() { return system_core.get_time(); });
mycsv.add_function("blade1 tip position [m]", [&]() {
    return turbine->get_elasto()->get_rna()->get_rotor()->get_blades()[0]
        ->get_nodes().back()->get_position();
});
```
