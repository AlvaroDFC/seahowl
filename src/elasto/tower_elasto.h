#ifndef TOWER_H_
#define TOWER_H_

#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;

struct TowerReferencePoint {
    ChVector<double> coordinates;
    double fraction;
    double density;
    double stiffness_axial;
    double stiffness_foreaft;
    double stiffness_sideside;
    double stiffness_torsion;
    DampingCoefficients damping_coefficients ;

    TowerReferencePoint operator*(const double factor) const {
        TowerReferencePoint new_point;
        new_point.coordinates = coordinates * factor;
        new_point.fraction = fraction * factor;
        new_point.density = density * factor;
        new_point.stiffness_axial = stiffness_axial * factor;
        new_point.stiffness_foreaft = stiffness_foreaft * factor;
        new_point.stiffness_sideside = stiffness_sideside * factor;
        new_point.stiffness_torsion = stiffness_torsion * factor;
        new_point.damping_coefficients.bx = damping_coefficients.bx * factor;
        new_point.damping_coefficients.bx = damping_coefficients.bx * factor;
        new_point.damping_coefficients.bx = damping_coefficients.bx * factor;
        new_point.damping_coefficients.bx = damping_coefficients.bx * factor;
        new_point.damping_coefficients.by = damping_coefficients.by * factor;
        new_point.damping_coefficients.bz = damping_coefficients.bz * factor;
        new_point.damping_coefficients.bt = damping_coefficients.bt * factor;
        new_point.damping_coefficients.alpha = damping_coefficients.alpha * factor;
        return new_point;
    };

    TowerReferencePoint operator+(const TowerReferencePoint& other) const {
        TowerReferencePoint new_point = *this;
        new_point.coordinates += other.coordinates;
        new_point.fraction += other.fraction;
        new_point.density += other.density;
        new_point.stiffness_axial += other.stiffness_axial;
        new_point.stiffness_foreaft += other.stiffness_foreaft;
        new_point.stiffness_sideside += other.stiffness_sideside;
        new_point.stiffness_torsion += other.stiffness_torsion;
        new_point.damping_coefficients.bx += other.damping_coefficients.bx;
        new_point.damping_coefficients.bx += other.damping_coefficients.bx;
        new_point.damping_coefficients.bx += other.damping_coefficients.bx;
        new_point.damping_coefficients.bx += other.damping_coefficients.bx;
        new_point.damping_coefficients.by += other.damping_coefficients.by;
        new_point.damping_coefficients.bz += other.damping_coefficients.bz;
        new_point.damping_coefficients.bt += other.damping_coefficients.bt;
        new_point.damping_coefficients.alpha += other.damping_coefficients.alpha;
        return new_point;
    };
};

class Tower {
  public:
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> elements;
    std::vector<TowerReferencePoint> reference_points;
    std::vector<TowerReferencePoint> discretized_points;
    std::vector<double> discretization_fractions;
    double height;
    double base_height;

    Tower() {}

    void build(std::shared_ptr<ChMesh> mesh);
    void build_nodes(std::shared_ptr<ChMesh> mesh);
    void build_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh);
    void translate(ChVector<double> translation_vector);
    void rotate(double angle, ChVector<double> axis);
    void set_damping_coefficients(double axial, double edge, double flap, double torsion);
    double get_mass();
};

#endif  // TOWER_H_
