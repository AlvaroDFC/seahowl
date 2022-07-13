#ifndef BLADE_ELASTO_H_
#define BLADE_ELASTO_H_

#include "reference_point_elasto.h"
#include "utils_elasto.h"
#include "../utils.h"

#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChSystemSMC.h"

using namespace chrono;
using namespace chrono::fea;

class BladeElasto {
  public:
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> elements;
    std::vector<std::shared_ptr<ChLoad<ChLoaderWeighted>>> loaders_aero;
    std::vector<double> discretization_fractions;
    std::vector<BladeReferencePointElasto> reference_points;
    std::vector<BladeReferencePointElasto> discretized_points;
    bool fpm_mode = false;

    BladeElasto();

    void build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh);
    void build_nodes(std::shared_ptr<ChMesh> mesh);
    void build_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh);
    void build_elements_tapered_timoshenko_fpm(std::shared_ptr<ChMesh> mesh);
    void build_loads(ChSystemSMC& system);
    void translate(ChVector<double> translation_vector);
    void rotate(double angle, ChVector<double> axis);
    void set_damping_coefficients(double axial, double edge, double flap, double torsion);
    double get_mass();
    void evaluate_position_rotation(ChVector<double>& position,
                                    ChQuaternion<double>& rotation,
                                    int element_index,
                                    double eta);
    void reset_loads();
    void accumulate_element_load(ChVector<double> load, int element_index, double eta);
};

#endif  // BLADE_ELASTO_H_
