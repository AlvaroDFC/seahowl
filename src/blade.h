#ifndef BLADE_H_
#define BLADE_H_

#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;

struct BladeReferencePoint {
    ChVector<double> coordinates;
    ChVector2<double> offset_elastic = ChVector2<double>(0.0, 0.0);
    ChVector2<double> offset_gravity = ChVector2<double>(0.0, 0.0);
    ChVector<double> fraction;
    double structural_twist;
    double density;
    double stiffness_axial;
    double stiffness_edge;
    double stiffness_flap;
    double stiffness_torsion;
    DampingCoefficients damping_coefficients = {0.001, 0.001, 0.001, 0.0};
};

class Blade {
  public:
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> elements;
    std::vector<BladeReferencePoint> reference_points;

    Blade();

    void make_blade(std::shared_ptr<ChMesh> mesh);
    void make_nodes(std::shared_ptr<ChMesh> mesh);
    void make_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh);
    void translate(ChVector<double> translation_vector);
    void rotate(double angle, ChVector<double> axis);
    void set_damping_coefficients(double axial, double edge, double flap, double torsion);
};

#endif  // BLADE_H_
