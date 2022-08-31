#pragma once

#include <memory>
#include <vector>

#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/core/reference_point.h>

namespace chrono {
namespace fea {
class ChMesh;
}
}  // namespace chrono

namespace seahowl {
namespace core {

class Tower {
  public:
    seahowl::elasto::TowerElasto elasto;  ///< Elastodynamic element mesh
    seahowl::aero::TowerAero aero;        ///< Aerodynamic element mesh
    std::vector<seahowl::core::TowerReferencePoint>
        reference_points;  ///<@todo  Refactor: Only used for construction to pass to elasto and aero. Use a Builder

    Tower();
    ~Tower();

    void build(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void set_discretization_elasto(std::vector<double> fractions);
    void set_discretization_aero(std::vector<double> fractions);
    void prestep(double time);
    void poststep(double time);
};

}  // namespace core
}  // namespace seahowl
