import seahowl
import pathlib

# change filepaths accordingly
# we set them relative to this script file for it to run from anywhere
thispath = pathlib.Path(__file__).parent.resolve()  # path of this file

# --------
# SEASTATE
# --------

# SeaState filepath (relative to this script)
ss_filepath = thispath / "../../../data/IEA15MW/env/SeaState_200m.dat"

# create SeaState instance
waves_ss = seahowl.env.SeaStateAdapter(str(ss_filepath))

# check outputs at different positions
positions = [
    [0.0, 0.0, 10.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, -5.0],
    [0.0, 0.0, -10.0],
    [0.0, 0.0, -15.0],
    [0.0, 0.0, -20.0],
]
for position in positions:
    density = waves_ss.get_density(position, 0.0)
    velocity = waves_ss.get_velocity(position, 0.0)
    acceleration = waves_ss.get_acceleration(position, 0.0)
    print(
        f"Fluid info at position {position}: density {density}, velocity {velocity}, acceleration {acceleration}"
    )
