import seahowl
import pathlib
import numpy as np

# change filepaths accordingly
# we set them relative to this script file for it to run from anywhere
thispath = pathlib.Path(__file__).parent.resolve()  # path of this file

# model options (change to actual filepaths)
blade_filepath = thispath / "../../../data/IEA15MW/base/blade.json"

# decay options
flapwise_force = -300.0  # initial flapwise force applied on blade tip
edgewise_force = 0.0  # initial edgewise force applied on blade tip


#  ____       _
# / ___|  ___| |_ _   _ _ __
# \___ \ / _ \ __| | | | '_ \
#  ___) |  __/ |_| |_| | |_) |
# |____/ \___|\__|\__,_| .__/
#                      |_|
#
# model setup

# log levels: critical, error, warn, info, debug, trace
seahowl.set_log_level_global("error")

# make simulation object
simulation = seahowl.core.Simulation()
simulation.dt = 0.01
simulation.duration = 5.0
simulation.outputs.dt_output = 0.0
simulation.outputs.has_gui = False
simulation.outputs.has_vtk = False
simulation.outputs.has_csv = True
simulation.outputs.set_output_folder("./output_blade_decay")

system_elasto = simulation.system_core.elasto

# make blade
blade_elasto = seahowl.elasto.BladeElastoFEA()
system_elasto.add(blade_elasto)  # add blade to elasto system
seahowl.io.populate_blade_elasto_from_file(str(blade_filepath), blade_elasto)
blade_elasto.fpm_mode = True

blade_elasto.build()

print(f"Number of elements: {len(blade_elasto.elements)}.")
blade_elasto.nodes[0].set_fixed(True)


#  ___       _ _   _       _ _          _   _
# |_ _|_ __ (_) |_(_) __ _| (_)______ _| |_(_) ___  _ __
#  | || '_ \| | __| |/ _` | | |_  / _` | __| |/ _ \| '_ \
#  | || | | | | |_| | (_| | | |/ / (_| | |_| | (_) | | | |
# |___|_| |_|_|\__|_|\__,_|_|_/___\__,_|\__|_|\___/|_| |_|
#
# initialization of model

# get equilibrium position
system_elasto.do_statics(True, 10)
pos_equilibrium = blade_elasto.nodes[-1].get_position()

# do statics
blade_elasto.nodes[-1].set_force(np.array([flapwise_force, edgewise_force, 0.0]), False)
system_elasto.do_statics(True, 10)
blade_elasto.nodes[-1].set_force(np.array([0.0, 0.0, 0.0]), False)

# make custom CSV
mycsv = simulation.outputs.create_new_csv("blade_info.csv")
mycsv.add_function("time [s]", lambda: system_elasto.get_time())
mycsv.add_function(
    "tip displacement [m]",
    lambda: blade_elasto.nodes[-1].get_position() - pos_equilibrium,
)


#  ____  _                 _       _   _
# / ___|(_)_ __ ___  _   _| | __ _| |_(_) ___  _ __
# \___ \| | '_ ` _ \| | | | |/ _` | __| |/ _ \| '_ \
#  ___) | | | | | | | |_| | | (_| | |_| | (_) | | | |
# |____/|_|_| |_| |_|\__,_|_|\__,_|\__|_|\___/|_| |_|
#
# simulation loop

# run simulation
print("Simulation started.")
simulation.initialize()
nsteps = int(simulation.duration / simulation.dt)
istep = 0
# run simulation
while simulation.system_core.get_time() < simulation.duration:
    if istep % (nsteps // 5) == 0:
        print(f"  - progress: {round(istep/nsteps*100)}% ({istep}/{nsteps} steps).")
    simulation.step()
    istep += 1
print("Simulation finished.")
