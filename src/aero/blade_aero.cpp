#include "blade_aero.h"

BladeElementAero::BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2) {
    properties = (point1 + point2) * 0.5;
    length = (point1.coordinates - point2.coordinates).Length();
}

ChVector2<double> BladeElementAero::get_induced_velocity(ChVector2<double>& local_velocity0) {
    // local_velocity_rotor0 is in rotor frame (unpitched and untwisted element)
    auto local_velocity_rotor0 = local_velocity0;
    double pitch_twist = pitch + properties.structural_twist;
    local_velocity_rotor0.Rotate(-pitch_twist);  // rotation is positive counter-clockwise
    // local_velocity is in local element frame
    ChVector2<double> local_velocity;

    double tol = 1e-3;
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
        double phi = atan2(local_velocity_rotor.y(), -local_velocity_rotor.x());
        // alpha = phi - pitch_twist;

        // reproject to blade element frame
        local_velocity = local_velocity_rotor;
        local_velocity.Rotate(pitch_twist);  // rotation is positive counter-clockwise

        // get coefficients from angle of attack
        alpha = atan2(local_velocity.y(), -local_velocity.x());
        // double phi = alpha + pitch_twist;

        if (phi == 0) {
            // avoid division by zero
            phi = 0.0001;
        }

        auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180.0 / CH_C_PI);

        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double cl = coefficients.lift;
        double cd = coefficients.drag;
        double cx = cl * cos_phi + cd * sin_phi;
        double cy = cl * sin_phi - cd * cos_phi;

        // update induction factors
        induction_factor_axial = chord_solidity / (4 * sin_phi * sin_phi) *
                                 (cx - chord_solidity * cy * cy / (4.0 * sin_phi * sin_phi)) * (1 - aa);
        induction_factor_tangential = chord_solidity * cy / (4 * sin_phi * cos_phi) * (1 + ap);

        // apply limits on induction factors
        if (induction_factor_axial < 0) {
            induction_factor_axial = 0;
        } else if (induction_factor_axial > 0.95) {
            induction_factor_axial = 0.95;
        }

        if (abs(induction_factor_axial - aa) < tol && abs(induction_factor_tangential - ap) < tol) {
            break;
        } else if (ii >= max_iter) {
            throw std::runtime_error(
                "Could not find new induction factor after " + std::to_string(ii) + " iterations. Last values: axial " +
                std::to_string(induction_factor_axial) + ", tangential " + std::to_string(induction_factor_tangential) +
                ". Previous values: axial " + std::to_string(aa) + ", tangential " + std::to_string(ap) +
                ". Alpha: " + std::to_string(alpha) + ", previous: " + std::to_string(alpha_previous) + ".");
        }
    }

    // return velocity in local
    return local_velocity;
}

void BladeAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        std::runtime_error("Not enough aero reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = get_discretized_points(discretization_fractions, reference_points);
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        // make element
        auto element = BladeElementAero(discretized_points[ii], discretized_points[ii + 1]);
        elements.push_back(element);
        // push empty load
        loads.push_back(ChVector<double>(0.0, 0.0, 0.0));
    }
}

void BladeAero::compute_wind_loads_bemt(WindModel& wind_model, double time) {
    double density = wind_model.get_density();
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        auto& properties = element.properties;

        // get fluid relative velocity
        auto wind_velocity = wind_model.get_wind_velocity(properties.coordinates, time);
        auto global_velocity = wind_velocity - properties.velocity;
        // project locally
        auto local_velocity0_3d = properties.rotation.RotateBack(global_velocity);
        // project using BEMT convention: x along chord, y along thickness up
        auto local_velocity0 = ChVector2<double>(local_velocity0_3d[1], -local_velocity0_3d[2]);

        // update induction factors and return local velocity
        auto local_velocity = element.get_induced_velocity(local_velocity0);

        // get coefficients from angle of attack
        double alpha = atan2(local_velocity.y(), -local_velocity.x());
        auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180 / CH_C_PI);

        // calculate drag and lift force
        double vel = local_velocity.Length();
        double chord = properties.chord;
        double length = element.length;
        double lift = 0.5 * density * vel * vel * chord * coefficients.lift * length;
        double drag = 0.5 * density * vel * vel * chord * coefficients.drag * length;

        // project to element local frame
        double lift_local = lift * cos(alpha) + drag * sin(alpha);
        double drag_local = lift * sin(alpha) - drag * cos(alpha);

        // transform from local to global load
        // use chrono convention
        auto load_local = ChVector<double>(0.0, lift_local, -drag_local);
        auto load_global = properties.rotation.Rotate(load_local);
        loads[ii] = load_global;
    }
}
