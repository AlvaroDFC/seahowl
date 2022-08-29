#pragma once
#include <seahowl/elasto/elasto.h>

#include <seahowl/elasto/utils_elasto.h>  // WeightedElasto

#include <chrono/physics/ChLoad.h>
#include <chrono/physics/ChLoaderU.h>

namespace chrono {
class ChSystemSMC;
namespace fea {
class ChMesh;
class ChNodeFEAxyzrot;
class ChElementBeamTaperedTimoshenko;
}  // namespace fea
}  // namespace chrono

namespace seahowl {
namespace elasto {

/**@brief Elastodynamic model for blade */
class BladeElasto : public ElastoFEAComponent {
  public:
    std::vector<std::shared_ptr<chrono::ChLoad<ChLoaderWeighted>>> loaders_aero;

    double pitch = 0;
    bool fpm_mode = false;

    BladeElasto();

    void build(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_nodes(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_elements_tapered_timoshenko(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_elements_tapered_timoshenko_fpm(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_loads(chrono::ChSystemSMC& system);

    virtual void set_damping_coefficients(double axial, double edge, double flap, double torsion) override;

    void evaluate_position_rotation(chrono::ChVector<double>& position,
                                    chrono::ChQuaternion<double>& rotation,
                                    int element_index,
                                    double eta);
    void reset_loads();
    void accumulate_element_load(chrono::ChVector<double> load, int element_index, double eta);
    void apply_pitch_increment(double pitch_increment);
};

}  // namespace elasto
}  // namespace seahowl