#ifndef READ_JSON_H_
#define READ_JSON_H_

#include <nlohmann/json.hpp>
#include "../core/blade_core.h"
#include "../core/rotor.h"
#include "../elasto/tower_elasto.h"
#include "../core/turbine.h"

std::vector<BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath);

Blade get_blade_from_json(std::string filepath);

std::vector<TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath);

TowerElasto get_tower_from_json(std::string filepath);

Rotor get_rotor_from_json(std::string filepath);

Turbine get_turbine_from_json(std::vector<std::string> filepaths_blades,
                              std::string filepath_rotor,
                              std::string filepath_tower);

#endif  // READ_JSON_H_
