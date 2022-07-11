#ifndef READ_JSON_H_
#define READ_JSON_H_

#include <nlohmann/json.hpp>
#include "../core/blade_core.h"
#include "../elasto/rotor.h"
#include "../elasto/tower.h"

std::vector<BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath);

Blade get_blade_from_json(std::string filepath);

std::vector<TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath);

Tower get_tower_from_json(std::string filepath);

Rotor get_rotor_from_json(std::string filepath);

#endif  // READ_JSON_H_
