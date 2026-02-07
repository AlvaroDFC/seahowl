import seahowl
import pathlib

# change filepaths accordingly
# we set them relative to this script file for it to run from anywhere
thispath = pathlib.Path(__file__).parent.resolve()  # path of this file

# ----------
# INFLOWWIND
# ----------

# InflowWind filepath (relative to this script)
ifw_filepath = thispath / "../../../data/IEA15MW/env/InflowWind.dat"

# create ConstantWind instance
wind_ifw = seahowl.env.InflowWindAdapter(str(ifw_filepath))

# set basic parameters
# density must be defined as InflowWind does not handle it internally
wind_ifw.air_density = 1.225  # set density
wind_ifw.zmin = 5.0  # below this z position, fluid velocity is enforced to 0


# check outputs at different positions
positions = [
    [0.0, 0.0, 150.0],
    [0.0, 0.0, 100.0],
    [0.0, 0.0, 10.0],
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0],
]
for position in positions:
    density = wind_ifw.get_density(position, 0.0)
    velocity = wind_ifw.get_velocity(position, 0.0)
    print(f"Fluid info at position {position}: density {density}, velocity {velocity}")
