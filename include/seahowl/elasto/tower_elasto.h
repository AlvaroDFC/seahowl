#pragma once

#include <seahowl/elasto/elasto.h>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

namespace chrono {
namespace fea {
class ChMesh;
}
}  // namespace chrono

namespace seahowl {
namespace elasto {


/**@brief Wind turbine tower elastodynamic model 

Implemented as Finite Element Beams
*/
class TowerElasto : public ElastoFEAComponent {
  public:

    std::vector<TowerReferencePoint> reference_points;
    std::vector<TowerReferencePoint> discretized_points;
    std::vector<double> discretization_fractions;
    double height;
    double base_height;

    TowerElasto() {}
    ~TowerElasto() {}

    void build(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_nodes(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_elements_tapered_timoshenko(std::shared_ptr<chrono::fea::ChMesh> mesh);


    virtual void set_damping_coefficients(double axial, double edge, double flap, double torsion) override;

};

}  // namespace elasto
}  // namespace seahowl
