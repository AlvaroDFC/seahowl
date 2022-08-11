#pragma once

namespace seahowl {
namespace core {
class Blade;
class Rotor;
class Turbine;
struct BladeReferencePoint;
}  // namespace core

namespace elasto {
class TowerElasto;  ///< @todo not Elasto hear
struct TowerReferencePoint;
}  // namespace elasto
}  // namespace seahowl


#include <vector>
#include <string>
#include <memory>

std::vector<seahowl::core::BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath);

seahowl::core::Blade get_blade_from_json(std::string filepath);

std::vector<seahowl::elasto::TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath);

seahowl::elasto::TowerElasto get_tower_from_json(std::string filepath);

seahowl::core::Rotor get_rotor_from_json(std::string filepath);

seahowl::core::Turbine get_turbine_from_json(std::vector<std::string> filepaths_blades,
                                             std::string filepath_rotor,
                                             std::string filepath_tower);
