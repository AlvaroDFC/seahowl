import seahowl


# -------------
# CONSTANT WIND
# -------------

# create ConstantWind instance
wind_constant = seahowl.env.ConstantWind()

# set basic parameters
wind_constant.air_density = 1.225  # set density
wind_constant.reference_height = 150.0  # set reference height for wind velocity
wind_constant.shear_coefficient = 0.12  # set shear coefficient

# define constant wind velocity
wind_constant.set_wind_velocity([12.0, 0.0, 0.0])  # set wind velocity


# ---------
# WIND RAMP
# ---------

# create WindRamp instance
wind_ramp = seahowl.env.WindRamp()

# set basic parameters
wind_ramp.air_density = 1.225  # set density
wind_ramp.reference_height = 150.0  # set reference height for wind velocity
wind_ramp.shear_coefficient = 0.12  # set shear coefficient

# define wind ramp
wind_ramp.set_wind_ramp(
    [12.0, 0.0, 0.0],  # velocity before ramp
    100.0,  # starting time of ramp
    [25.0, 0.0, 0.0],  # velocity after ramp
    500.0,  # ending time of ramp
)
