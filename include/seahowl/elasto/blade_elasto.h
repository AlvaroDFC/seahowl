#pragma once

struct BladeReferencePointElasto;
#include <seahowl/elasto/utils_elasto.h> // WeightedElasto

#include <chrono/physics/ChLoad.h>
#include <chrono/physics/ChLoaderU.h>

namespace chrono {
class ChSystemSMC;
namespace fea {
class ChMesh;
class ChNodeFEAxyzrot;
class ChElementBeamTaperedTimoshenko;
}  // namespace fea
} 
    // namespace chrono
    /**@brief Elastodynamic model for blade */
class BladeElasto {
  public:
    std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> nodes;
    std::vector<std::shared_ptr<chrono::fea::ChElementBeamTaperedTimoshenko>> elements;
    std::vector<std::shared_ptr<chrono::ChLoad<ChLoaderWeighted>>> loaders_aero;
    std::vector<double> discretization_fractions;
    std::vector<BladeReferencePointElasto> reference_points;
    std::vector<BladeReferencePointElasto> discretized_points;
    double pitch = 0;
    bool fpm_mode = false;

    BladeElasto();

    void build(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_nodes(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_elements_tapered_timoshenko(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_elements_tapered_timoshenko_fpm(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build_loads(chrono::ChSystemSMC& system);
    void translate(chrono::ChVector<double> translation_vector);
    void rotate(double angle, chrono::ChVector<double> axis);
    void set_damping_coefficients(double axial, double edge, double flap, double torsion);
    double get_mass();
    void evaluate_position_rotation(chrono::ChVector<double>& position,
                                    chrono::ChQuaternion<double>& rotation,
                                    int element_index,
                                    double eta);
    void reset_loads();
    void accumulate_element_load(chrono::ChVector<double> load, int element_index, double eta);
    void apply_pitch_increment(double pitch_increment);
};

