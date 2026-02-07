import seahowl
import pathlib

# change filepaths accordingly
# we set them relative to this script file for it to run from anywhere
thispath = pathlib.Path(__file__).parent.resolve()  # path of this file

# ---------
# ENV MODEL
# ---------

# environment model JSON input file (relative to this script)
env_filepath = thispath / "../../../data/IEA15MW/env/env_waves_200m_seastate.json"

# make environment model
myenv = seahowl.io.get_environmental_model_from_file(str(env_filepath))

# check outputs at different positions
positions = [
    [0.0, 0.0, 150.0],
    [0.0, 0.0, 100.0],
    [0.0, 0.0, 10.0],
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 0.0, -10.0],
    [0.0, 0.0, -20.0],
]
for position in positions:
    density = myenv.fluid_models.get_density(position, 0.0)
    velocity = myenv.fluid_models.get_velocity(position, 0.0)
    print(f"Fluid info at position {position}: density {density}, velocity {velocity}")
