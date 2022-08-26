#include "seahowl/aero/blade_aero.h"

using seahowl::aero::BladeElementAero;
using seahowl::aero::BladeAero;

BladeElementAero::BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2) {
    properties = (point1 + point2) * 0.5;
    length = (point1.m_coordinates - point2.m_coordinates).Length();
}

BladeElementAero::~BladeElementAero() {}

chrono::ChVector2<double> BladeElementAero::get_induced_velocity_rotor(chrono::ChVector2<double>& local_velocity_rotor0,
                                                                       size_t nblades,
                                                                       bool tip_loss,
                                                                       bool hub_loss) {
    // local_velocity is in local element frame
    chrono::ChVector2<double> local_velocity;
    chrono::ChVector2<double> local_velocity_rotor;
    double pitch_twist = pitch + properties.structural_twist;

    double tol_rel = 1e-3;
    double tol_abs = 1e-3;
    int max_iter = 100;
    double alpha = -1000.0;
    double alpha_previous;
    auto aa = induction_factor_axial;
    auto ap = induction_factor_tangential;
    // limits
    double aa_max = 1.0;
    double aa_min = 0.0;
    double ap_max = 1.5;
    double ap_min = 0.0;
    for (int ii = 1; ii <= max_iter; ii++) {
        // store previous alpha
        alpha_previous = alpha;
        // store previous induction factors
        auto aa_previous = aa;
        auto ap_previous = ap;

        // local velocity updated with induction factors
        local_velocity_rotor =
            chrono::ChVector2<double>(local_velocity_rotor0.x() * (1.0 + ap), local_velocity_rotor0.y() * (1.0 - aa));
        double phi = atan2(local_velocity_rotor.y(), -local_velocity_rotor.x());
        alpha = phi - pitch_twist;
        // check that alpha is still in range
        if (alpha < -chrono::CH_C_PI || alpha > chrono::CH_C_PI) {
            alpha = abs(std::fmod((alpha + 3 * chrono::CH_C_PI), 2 * chrono::CH_C_PI)) - chrono::CH_C_PI;
        }

        // // other way to get alpha and phi
        // // reproject to blade element frame
        // local_velocity = local_velocity_rotor;
        // local_velocity.Rotate(pitch_twist);  // rotation is positive counter-clockwise
        // // get coefficients from angle of attack
        // alpha = atan2(local_velocity.y(), -local_velocity.x());
        // double phi = alpha + pitch_twist;

        // get aero coefficients
        auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180.0 / chrono::CH_C_PI);

        // get drag and lift coefficients
        auto cl = coefficients.lift;
        auto cd = coefficients.drag;
        // projected to rotor local frame
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double cn = cl * cos_phi + cd * sin_phi;
        double ct = cl * sin_phi - cd * cos_phi;

        // losses
        double loss_factor = 1.0;
        if (tip_loss) {
            // Prandtl's approximation for tip-loss factor
            loss_factor *= (2.0 / chrono::CH_C_PI) *
                           std::acos(std::exp(nblades * (-distance_from_tip)) / (2.0 * radius * std::fabs(sin_phi)));
        }
        if (hub_loss) {
            // hub loss
            double hub_radius = (radius - distance_from_hub);
            loss_factor *= (2.0 / chrono::CH_C_PI) * std::acos(std::exp(nblades * (-distance_from_hub)) /
                                                               (2.0 * hub_radius * std::fabs(sin_phi)));
        }

        // update induction factors

        // axial induction, based on AeroDyn v15 implementation
        double kk = chord_solidity * cn / (4.0 * pow(sin_phi, 2));
        if (kk <= 2.0 / 3.0) {
            if (kk == -1.0) {
                double temp = -aa_max * (1.0 + kk);
                aa = (temp > 0.0) - (temp < 0.0);  // sign of temp
            } else {
                aa = kk / (1.0 + kk);
            }
            if (kk < -1.0) {
                throw std::runtime_error("Not valid BEMT solution for axial induction.");
            }
        } else {
            double ff = loss_factor;
            double temp = 2.0 * ff * kk;
            double g1 = temp - (10.0 / 9.0 - ff);
            double g2 = temp - (4.0 / 3.0 - ff) * ff;
            double g3 = temp - (25.0 / 9.0 - 2.0 * ff);

            if (abs(g3) < 1e-6) {
                aa = 1.0 - 0.5 / sqrt(g2);
            } else {
                aa = (g1 - sqrt(abs(g2))) / g3;
            }
        }

        // tangential induction
        ap = chord_solidity * ct / (4.0 * sin_phi * cos_phi * loss_factor) * (1.0 + ap);

        // apply limits on induction factors
        if (aa > aa_max) {
            aa = aa_max;
        } else if (aa < aa_min) {
            aa = aa_min;
        }
        if (ap < ap_min) {
            ap = ap_min;
        } else if (ap > ap_max) {
            ap = ap_max;
        }

        if ((fabs(aa - aa_previous) <= tol_rel * fabs(std::max(aa_previous, aa))) &&
                (fabs(ap - ap_previous) <= tol_rel * fabs(std::max(ap_previous, aa))) ||
            (fabs(aa - aa_previous) <= tol_abs) && (fabs(ap - ap_previous) <= tol_abs)) {
            break;
        } else if (ii >= max_iter) {
            std::cout << "Warning: could not converge to new induction factor after " + std::to_string(ii) +
                             " iterations. Axial: " + std::to_string(induction_factor_axial) +
                             ", previous:" + std::to_string(aa) +
                             ". Tangential: " + std::to_string(induction_factor_tangential) + ", previous " +
                             std::to_string(ap) + ". Alpha: " + std::to_string(alpha) +
                             ", previous: " + std::to_string(alpha_previous) + ". Local velocity in: (" +
                             std::to_string(local_velocity_rotor0.x()) + ", " +
                             std::to_string(local_velocity_rotor0.y()) + ")."
                      << std::endl;
            std::cout << phi * 180 / chrono::CH_C_PI << " " << cos_phi << " " << sin_phi;
            // throw std::runtime_error(
            //     "Could not find new induction factor after " + std::to_string(ii) + " iterations. Axial: " +

            //     std::to_string(induction_factor_axial) + ", previous:" + std::to_string(aa) +
            //     ". Tangential: " + std::to_string(induction_factor_tangential) + ", previous " + std::to_string(ap) +
            //     ". Alpha: " + std::to_string(alpha) + ", previous: " + std::to_string(alpha_previous) + ".");
        }
    }

    // store induction factors for starting point of next time iteration
    induction_factor_axial = aa;
    induction_factor_tangential = ap;

    // return velocity in local
    return local_velocity_rotor;
}

// // DO NOT USE: use get_inducted_velocity_rotor instead
// // method is kept here for testing purposes
// ChVector2<double> BladeElementAero::get_induced_velocity_element(ChVector2<double>& local_velocity0) {
//     // local_velocity_rotor0 is in rotor frame (unpitched and untwisted element)
//     auto local_velocity_rotor0 = local_velocity0;
//     double pitch_twist = pitch + properties.structural_twist;
//     local_velocity_rotor0.Rotate(-pitch_twist);  // rotation is positive counter-clockwise
//     // local_velocity is in local element frame
//     ChVector2<double> local_velocity;

//     double tol = 1e-3;
//     double max_iter = 100;
//     double alpha = -1000.0;
//     double alpha_previous;
//     for (int ii = 1; ii <= max_iter; ii++) {
//         alpha_previous = alpha;
//         auto aa = induction_factor_axial;
//         auto ap = induction_factor_tangential;

//         // local velocity updated with induction factors
//         auto local_velocity_rotor =
//             ChVector2<double>(local_velocity_rotor0.x() * (1.0 + ap), local_velocity_rotor0.y() * (1.0 - aa));
//         double phi = atan2(local_velocity_rotor.y(), -local_velocity_rotor.x());
//         // alpha = phi - pitch_twist;

//         // reproject to blade element frame
//         local_velocity = local_velocity_rotor;
//         local_velocity.Rotate(pitch_twist);  // rotation is positive counter-clockwise

//         // get coefficients from angle of attack
//         alpha = atan2(local_velocity.y(), -local_velocity.x());
//         // double phi = alpha + pitch_twist;

//         if (phi == 0) {
//             // avoid division by zero
//             phi = 0.0001;
//         }

//         auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180.0 / CH_C_PI);

//         double cos_phi = cos(phi);
//         double sin_phi = sin(phi);
//         double cl = coefficients.lift;
//         double cd = coefficients.drag;
//         double cx = cl * cos_phi + cd * sin_phi;
//         double cy = cl * sin_phi - cd * cos_phi;

//         // update induction factors
//         induction_factor_axial = chord_solidity / (4 * sin_phi * sin_phi) *
//                                  (cx - chord_solidity * cy * cy / (4.0 * sin_phi * sin_phi)) * (1 - aa);
//         induction_factor_tangential = chord_solidity * cy / (4 * sin_phi * cos_phi) * (1 + ap);

//         // apply limits on induction factors
//         if (induction_factor_axial < 0) {
//             induction_factor_axial = 0;
//         } else if (induction_factor_axial > 0.95) {
//             induction_factor_axial = 0.95;
//         }

//         if (abs(induction_factor_axial - aa) < tol && abs(induction_factor_tangential - ap) < tol) {
//             break;
//         } else if (ii >= max_iter) {
//             throw std::runtime_error(
//                 "Could not find new induction factor after " + std::to_string(ii) + " iterations. Last values: axial
//                 " + std::to_string(induction_factor_axial) + ", tangential " +
//                 std::to_string(induction_factor_tangential) +
//                 ". Previous values: axial " + std::to_string(aa) + ", tangential " + std::to_string(ap) +
//                 ". Alpha: " + std::to_string(alpha) + ", previous: " + std::to_string(alpha_previous) + ".");
//         }
//     }

//     // return velocity in local
//     return local_velocity;
// }

BladeAero::BladeAero() {}

BladeAero::~BladeAero() {}

void BladeAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        // make element
        auto element = BladeElementAero(discretized_points[ii], discretized_points[ii + 1]);
        elements.push_back(element);
        // push empty load
        loads.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
    }

    // get distance from tip
    compute_distances_from_tip();
}

void BladeAero::compute_distances_from_tip() {
    // this is the position of the element at the tip
    auto& element_tip_position = elements.back().properties.m_coordinates;
    // need to add 0.5*length of the element to get actual distance from tip
    double offset = 0.5 * elements.back().length;
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        element.distance_from_tip = (element.properties.m_coordinates - element_tip_position).Length() + offset;
    }
}

void BladeAero::compute_distances_from_hub(chrono::ChVector<double> hub_apex_position, double hub_radius) {
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        element.distance_from_hub = (element.properties.m_coordinates - hub_apex_position).Length() - hub_radius;
    }
}

void BladeAero::compute_radii(chrono::ChVector<double> hub_apex_position) {
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        element.radius = (element.properties.m_coordinates - hub_apex_position).Length();
    }
}

// // DO NOT USE: use compute_wind_loads_bemt from RotorAero class instead
// // method is kept here for testing purposes
// void BladeAero::compute_wind_loads_bemt(WindModel& wind_model, double time) {
//     double density = wind_model.get_density();
//     for (int ii = 0; ii < elements.size(); ii++) {
//         auto& element = elements[ii];
//         auto& properties = element.properties;

//         // get fluid relative velocity
//         auto wind_velocity = wind_model.get_wind_velocity(properties.coordinates, time);
//         auto global_velocity = wind_velocity - properties.velocity;
//         // project locally
//         auto local_velocity0_3d = properties.rotation.RotateBack(global_velocity);
//         // project using BEMT convention: x along chord, y along thickness up
//         auto local_velocity0 = ChVector2<double>(local_velocity0_3d[1], -local_velocity0_3d[2]);

//         // update induction factors and return local velocity
//         auto local_velocity = element.get_induced_velocity_element(local_velocity0);

//         // get coefficients from angle of attack
//         double alpha = atan2(local_velocity.y(), -local_velocity.x());
//         auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180 / CH_C_PI);

//         // calculate drag and lift force
//         double vel = local_velocity.Length();
//         double chord = properties.chord;
//         double length = element.length;
//         double lift = 0.5 * density * vel * vel * chord * coefficients.lift * length;
//         double drag = 0.5 * density * vel * vel * chord * coefficients.drag * length;

//         // project to element local frame
//         double lift_local = lift * cos(alpha) + drag * sin(alpha);
//         double drag_local = lift * sin(alpha) - drag * cos(alpha);

//         // transform from local to global load
//         // use chrono convention
//         auto load_local = ChVector<double>(0.0, lift_local, -drag_local);
//         auto load_global = properties.rotation.Rotate(load_local);
//         loads[ii] = load_global;
//     }
// }
