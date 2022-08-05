#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/elasto/tower_elasto.h>

#include <vector>

/**@brief Seahowl base namespace */
namespace seahowl {

/**@brief Seahowl core module */
namespace core {

/**@brief The turbine (without support and foundations) */
class Turbine {
  public:
    std::vector<std::shared_ptr<Blade>> m_blades;
    Rotor m_rotor;
    TowerElasto m_tower;

    Turbine();

    ~Turbine() {}

    void build(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh);
    void prestep(double time);
    void poststep(double time);
    void translate(chrono::ChVector<double> translation_vector);
    void rotate(double angle, chrono::ChVector<double> axis);
    void compute_wind_loads(WindModel& wind_model, double time);
};

}  // namespace core
}  // namespace seahowl
