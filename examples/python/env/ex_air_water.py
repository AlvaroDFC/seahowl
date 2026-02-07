import seahowl

# make environment model
myenv = seahowl.env.EnvModel()

# make wind model
wind = seahowl.env.ConstantWind()
wind.air_density = 1.225
wind.set_wind_velocity([12.0, 0.0, 0.0])
wind.reference_height = 150.0
wind.shear_coefficient = 0.12
myenv.add_model(wind)

# make water model
water = seahowl.env.StillWater()
water.density = 1025.0
water.mean_water_level = 0.0
water.water_depth = 50.0
myenv.add_model(water)


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
