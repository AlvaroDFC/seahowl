#ifndef BLADE_AERO_H_
#define BLADE_AERO_H_

#include "reference_point_aero.h"
#include "wind_models.h"
#include "../utils.h"

#include "chrono/core/ChVector.h"

using namespace chrono;

struct BladeElementAero {
    BladeReferencePointAero properties;
    double length;
    double pitch_beta = 0.0;
    double swept_annulus = 1.0;
    double chord_solidity = 1.0;
    double induction_factor_axial = 0.0;
    double induction_factor_tangential = 0.0;

    BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2) {
        properties = (point1 + point2) * 0.5;
        length = (point1.coordinates - point2.coordinates).Length();
    }

    ~BladeElementAero() {}

    ChVector2<double> get_induced_velocity(ChVector2<double>& local_velocity0) {
        // local_velocity_rotor0 is in rotor frame (unpitched and untwisted element)
        auto local_velocity_rotor0 = local_velocity0;
        local_velocity_rotor0.Rotate(pitch_beta);
        // local_velocity is in local element frame
        ChVector2<double> local_velocity;

        double tol = 1e-6;
        double max_iter = 100;
        double alpha = -1000.0;
        double alpha_previous;
        for (int ii = 1; ii <= max_iter; ii++) {
            alpha_previous = alpha;
            auto aa = induction_factor_axial;
            auto ap = induction_factor_tangential;

            // local velocity updated with induction factors
            auto local_velocity_rotor =
                ChVector2<double>(local_velocity_rotor0.x() * (1.0 + ap), local_velocity_rotor0.y() * (1.0 - aa));
            // reproject to blade element frame
            local_velocity = local_velocity_rotor;
            local_velocity.Rotate(-pitch_beta);

            // get coefficients from angle of attack
            alpha = atan2(local_velocity.y(), local_velocity.x());
            auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180.0 / CH_C_PI);

            double phi = alpha + pitch_beta;
            double cos_phi = cos(phi);
            double sin_phi = sin(phi);
            double cl = coefficients.lift;
            double cd = coefficients.drag;
            double cx = cl * cos_phi + cd * sin_phi;
            double cy = cl * sin_phi - cd * cos_phi;

            // update induction factors
            // TODO: apply limits on induction factors
            induction_factor_axial = chord_solidity / (4 * sin_phi * sin_phi) *
                                     (cx - chord_solidity * cy * cy / (4.0 * sin_phi * sin_phi)) * (1 - aa);
            induction_factor_tangential = chord_solidity * cy / (4 * sin_phi * cos_phi) * (1 + ap);

            double tol = 1e-6;
            if (abs(alpha - alpha_previous) < tol) {
                break;
            } else if (ii >= max_iter) {
                throw std::runtime_error("Could not find new induction factor after " + std::to_string(ii) +
                                         " iterations. Last values: axial " + std::to_string(induction_factor_axial) +
                                         ", tangential " + std::to_string(induction_factor_tangential) +
                                         ". Previous values: axial " + std::to_string(aa) + ", tangential " +
                                         std::to_string(ap));
            }
        }

        // return velocity in local
        return local_velocity;
    }
};

class BladeAero {
  public:
    std::vector<double> discretization_fractions;
    std::vector<BladeReferencePointAero> reference_points;
    std::vector<BladeReferencePointAero> discretized_points;
    std::vector<BladeElementAero> elements;
    std::vector<ChVector<double>> loads;

    BladeAero() {}
    ~BladeAero() {}

    void build();
    void compute_loads(double time, WindModel& wind_model);
};

#endif  // BLADE_AERO_H_
