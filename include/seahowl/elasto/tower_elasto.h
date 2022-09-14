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
    std::vector<TowerReferencePointElasto> reference_points;
    std::vector<TowerReferencePointElasto> discretized_points;
    std::vector<double> discretization_fractions;
    double height;
    double base_height;

    TowerElasto();
    ~TowerElasto();

    void assemble(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build();
    void build_nodes();
    void build_elements_tapered_timoshenko();

    virtual void set_damping_coefficients(double axial, double edge, double flap, double torsion) override;
    void evaluate_position_rotation(chrono::ChVector<double>& position,
                                    chrono::ChQuaternion<double>& rotation,
                                    int element_index,
                                    double eta);
    void reset_loads();
    void accumulate_element_load(chrono::ChVector<double> load, int element_index, double eta);
};

}  // namespace elasto
}  // namespace seahowl
