#ifndef BLADE_H_
#define BLADE_H_

#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChSystemSMC.h"
#include "utils.h"

using namespace chrono;
using namespace chrono::fea;

struct BladeReferencePoint {
    ChVector<double> coordinates = ChVector<double>(0.0, 0.0, 0.0);
    ChVector2<double> offset_elastic = ChVector2<double>(0.0, 0.0);
    ChVector2<double> offset_gravity = ChVector2<double>(0.0, 0.0);
    ChMatrixNM<double, 6, 6> stiffness_matrix;
    ChMatrixNM<double, 6, 6> mass_matrix;
    double fraction = 0.0;
    double structural_twist = 0.0;
    DampingCoefficients damping_coefficients;

    BladeReferencePoint() {
        stiffness_matrix.setZero();
        mass_matrix.setZero();
        damping_coefficients.bx = 0.0;
        damping_coefficients.by = 0.0;
        damping_coefficients.bz = 0.0;
        damping_coefficients.bt = 0.0;
        damping_coefficients.alpha = 0.0;
    }

    ~BladeReferencePoint() {}

    BladeReferencePoint operator*(const double factor) const {
        BladeReferencePoint new_point = *this;
        new_point.coordinates *= factor;
        new_point.offset_elastic *= factor;
        new_point.offset_gravity *= factor;
        new_point.fraction *= factor;
        new_point.structural_twist *= factor;
        new_point.mass_matrix *= factor;
        new_point.stiffness_matrix *= factor;
        new_point.damping_coefficients.bx *= factor;
        new_point.damping_coefficients.by *= factor;
        new_point.damping_coefficients.bz *= factor;
        new_point.damping_coefficients.bt *= factor;
        new_point.damping_coefficients.alpha *= factor;
        return new_point;
    };
    BladeReferencePoint operator+(const BladeReferencePoint& other) const {
        BladeReferencePoint new_point = *this;
        new_point.coordinates += other.coordinates;
        new_point.offset_elastic += other.offset_elastic;
        new_point.offset_gravity += other.offset_gravity;
        new_point.fraction += other.fraction;
        new_point.structural_twist += other.structural_twist;
        new_point.stiffness_matrix += other.stiffness_matrix;
        new_point.mass_matrix += other.mass_matrix;
        new_point.damping_coefficients.bx += other.damping_coefficients.bx;
        new_point.damping_coefficients.by += other.damping_coefficients.by;
        new_point.damping_coefficients.bz += other.damping_coefficients.bz;
        new_point.damping_coefficients.bt += other.damping_coefficients.bt;
        new_point.damping_coefficients.alpha += other.damping_coefficients.alpha;
        return new_point;
    };
};

struct BladeAeroReferencePoint {
    double fraction;
    ChVector<double> coordinates;
    ChVector<double> direction_x;
    ChVector<double> direction_y;
    ChVector<double> velocity;

    BladeAeroReferencePoint operator*(const double factor) const {
        BladeAeroReferencePoint new_point;
        new_point.fraction = fraction * factor;
        new_point.coordinates = coordinates * factor;
        new_point.direction_x = direction_x * factor;
        new_point.direction_y = direction_y * factor;
        new_point.velocity = velocity * factor;
        return new_point;
    };
    BladeAeroReferencePoint operator+(const BladeAeroReferencePoint& other) const {
        BladeAeroReferencePoint new_point = *this;
        new_point.fraction += other.fraction;
        new_point.coordinates += other.coordinates;
        new_point.direction_x += other.direction_x;
        new_point.direction_y += other.direction_y;
        new_point.velocity += other.velocity;
        return new_point;
    };
};

class Blade {
  public:
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> elements;
    std::vector<std::shared_ptr<ChLoad<ChLoaderWeighted>>> loaders_aero;
    std::vector<BladeReferencePoint> reference_points;
    std::vector<BladeReferencePoint> discretized_points;
    std::vector<double> discretization_fractions;
    std::vector<double> discretization_aero;
    bool fpm_mode = false;

    Blade();

    void build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh);
    void build_nodes(std::shared_ptr<ChMesh> mesh);
    void build_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh);
    void build_elements_tapered_timoshenko_fpm(std::shared_ptr<ChMesh> mesh);
    void build_loads(ChSystemSMC& system);
    void translate(ChVector<double> translation_vector);
    void rotate(double angle, ChVector<double> axis);
    void set_damping_coefficients(double axial, double edge, double flap, double torsion);
    std::vector<BladeAeroReferencePoint> get_aerodynamic_point_positions();
    double get_mass();
};

#endif  // BLADE_H_
