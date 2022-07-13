#ifndef REFERENCE_POINT_AERO_H_
#define REFERENCE_POINT_AERO_H_

#include "airfoil.h"

struct BladeReferencePointAero {
    double fraction;
    ChVector<double> coordinates;
    ChQuaternion<double> rotation;
    ChVector<double> direction_x;
    ChVector<double> direction_y;
    ChVector<double> velocity;
    std::vector<AirfoilProperties> airfoil_properties;

    BladeReferencePointAero(BladeReferencePoint point) {
        coordinates = point.coordinates;
        direction_x = ChVector<double>(0.0, 0.0, 0.0);
        direction_y = ChVector<double>(0.0, 0.0, 0.0);
        airfoil_properties = point.airfoil_properties;
    }

    BladeReferencePointAero operator*(const double factor) const {
        BladeReferencePointAero new_point = *this;
        new_point.fraction *= factor;
        new_point.coordinates *= factor;
        new_point.direction_x *= factor;
        new_point.direction_y *= factor;
        new_point.velocity *= factor;
        for (int ii = 0; ii < airfoil_properties.size(); ii++) {
            new_point.airfoil_properties[ii] = airfoil_properties[ii] * factor;
        }
        return new_point;
    };
    BladeReferencePointAero operator+(const BladeReferencePointAero& other) const {
        BladeReferencePointAero new_point = *this;
        new_point.fraction += other.fraction;
        new_point.coordinates += other.coordinates;
        new_point.direction_x += other.direction_x;
        new_point.direction_y += other.direction_y;
        new_point.velocity += other.velocity;
        for (int ii = 0; ii < airfoil_properties.size(); ii++) {
            if (airfoil_properties[ii].reynolds_number != other.airfoil_properties[ii].reynolds_number) {
                throw std::runtime_error("Trying to add airfoil properties with different Reynolds number.");
            }
            new_point.airfoil_properties[ii] = airfoil_properties[ii] + other.airfoil_properties[ii];
        }
        return new_point;
    };
};

#endif  // REFERENCE_POINT_AERO_H_
