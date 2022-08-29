#include <seahowl/elasto/reference_point_elasto.h>

using namespace seahowl::elasto;

BladeReferencePointElasto::BladeReferencePointElasto() {}

BladeReferencePointElasto::BladeReferencePointElasto(seahowl::core::BladeReferencePoint point) {
    m_coordinates = point.m_coordinates;
    m_offset_elastic = point.m_offset_elastic;
    m_offset_gravity = point.m_offset_gravity;
    stiffness_matrix = point.stiffness_matrix;
    mass_matrix = point.mass_matrix;
    fraction = point.fraction;
    structural_twist = point.structural_twist;
    damping_coefficients = point.damping_coefficients;
}

BladeReferencePointElasto::~BladeReferencePointElasto() {}

BladeReferencePointElasto BladeReferencePointElasto::operator*(const double factor) const {
    BladeReferencePointElasto new_point = *this;
    new_point.m_coordinates *= factor;
    new_point.m_offset_elastic *= factor;
    new_point.m_offset_gravity *= factor;
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

BladeReferencePointElasto BladeReferencePointElasto::operator+(const BladeReferencePointElasto& other) const {
    BladeReferencePointElasto new_point = *this;
    new_point.m_coordinates += other.m_coordinates;
    new_point.m_offset_elastic += other.m_offset_elastic;
    new_point.m_offset_gravity += other.m_offset_gravity;
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

TowerReferencePoint TowerReferencePoint::operator*(const double factor) const {
    TowerReferencePoint new_point;
    new_point.coordinates = coordinates * factor;
    new_point.fraction = fraction * factor;
    new_point.density = density * factor;
    new_point.stiffness_axial = stiffness_axial * factor;
    new_point.stiffness_foreaft = stiffness_foreaft * factor;
    new_point.stiffness_sideside = stiffness_sideside * factor;
    new_point.stiffness_torsion = stiffness_torsion * factor;
    new_point.damping_coefficients.bx = damping_coefficients.bx * factor;
    new_point.damping_coefficients.by = damping_coefficients.by * factor;
    new_point.damping_coefficients.bz = damping_coefficients.bz * factor;
    new_point.damping_coefficients.bt = damping_coefficients.bt * factor;
    new_point.damping_coefficients.alpha = damping_coefficients.alpha * factor;
    return new_point;
};

TowerReferencePoint TowerReferencePoint::operator+(const TowerReferencePoint& other) const {
    TowerReferencePoint new_point = *this;
    new_point.coordinates += other.coordinates;
    new_point.fraction += other.fraction;
    new_point.density += other.density;
    new_point.stiffness_axial += other.stiffness_axial;
    new_point.stiffness_foreaft += other.stiffness_foreaft;
    new_point.stiffness_sideside += other.stiffness_sideside;
    new_point.stiffness_torsion += other.stiffness_torsion;
    new_point.damping_coefficients.bx += other.damping_coefficients.bx;
    new_point.damping_coefficients.by += other.damping_coefficients.by;
    new_point.damping_coefficients.bz += other.damping_coefficients.bz;
    new_point.damping_coefficients.bt += other.damping_coefficients.bt;
    new_point.damping_coefficients.alpha += other.damping_coefficients.alpha;
    return new_point;
};
