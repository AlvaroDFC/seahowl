#pragma once

#include <nlohmann/json.hpp>
#include <seahowl/core/blade.h>

#include <seahowl/core/rotor.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/turbine.h>

#include <vector>
#include <string>
#include <memory>

std::vector<BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath);

seahowl::core::Blade get_blade_from_json(std::string filepath);

std::vector<TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath);

TowerElasto get_tower_from_json(std::string filepath);

Rotor get_rotor_from_json(std::string filepath);

seahowl::core::Turbine get_turbine_from_json(std::vector<std::string> filepaths_blades,
                              std::string filepath_rotor,
                              std::string filepath_tower);


