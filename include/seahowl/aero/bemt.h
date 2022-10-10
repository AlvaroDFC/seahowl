#pragma once

#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>

#include <chrono/core/ChVector.h>
#include <chrono/core/ChVector2.h>

namespace seahowl {
namespace aero {

chrono::ChVector2<double> get_induced_velocity(BladeElementAero& element,
                                               const chrono::ChVector2<double>& local_velocity_rotor0,
                                               size_t nblades = 3,
                                               bool tip_loss = true,
                                               bool hub_loss = true);

void apply_tower_shadow_effect_on_wind(chrono::ChVector<double>& wind_velocity,
                                       const chrono::ChVector<double>& position,
                                       double blade_azimuth,
                                       const TowerAero& tower_aero);
}  // namespace aero
}  // namespace seahowl
