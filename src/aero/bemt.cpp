#include <seahowl/aero/bemt.h>

#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/core/utils.h>

#include <chrono/core/ChVector.h>
#include <chrono/core/ChVector2.h>

chrono::ChVector2<double> seahowl::aero::get_induced_velocity(seahowl::aero::BladeElementAero& element,
                                                              const chrono::ChVector2<double>& local_velocity_rotor0,
                                                              size_t nblades,
                                                              bool tip_loss,
                                                              bool hub_loss) {
    // local_velocity is in local element frame
    chrono::ChVector2<double> local_velocity;
    chrono::ChVector2<double> local_velocity_rotor;
    double pitch_twist = element.pitch + element.properties.structural_twist;

    double tol_rel = 1e-3;
    double tol_abs = 1e-3;
    int max_iter = 100;
    double alpha = -1000.0;
    double alpha_previous;
    auto aa = element.induction_factor_axial;
    auto ap = element.induction_factor_tangential;
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
        auto coefficients = element.properties.airfoil_properties[0].find_coefficients(alpha * 180.0 / chrono::CH_C_PI);

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
            loss_factor *= (2.0 / chrono::CH_C_PI) * std::acos(std::exp(nblades * (-element.distance_from_tip)) /
                                                               (2.0 * element.radius * std::fabs(sin_phi)));
        }
        if (hub_loss) {
            // hub loss
            double hub_radius = (element.radius - element.distance_from_hub);
            loss_factor *= (2.0 / chrono::CH_C_PI) * std::acos(std::exp(nblades * (-element.distance_from_hub)) /
                                                               (2.0 * hub_radius * std::fabs(sin_phi)));
        }

        // update induction factors

        // axial induction, based on AeroDyn v15 implementation
        double kk = element.chord_solidity * cn / (4.0 * pow(sin_phi, 2));
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
        ap = element.chord_solidity * ct / (4.0 * sin_phi * cos_phi * loss_factor) * (1.0 + ap);

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
                             " iterations. Axial: " + std::to_string(aa) + ", previous:" + std::to_string(aa_previous) +
                             ". Tangential: " + std::to_string(ap) + ", previous " + std::to_string(ap_previous) +
                             ". Alpha: " + std::to_string(alpha) + ", previous: " + std::to_string(alpha_previous) +
                             ". Local velocity in: (" + std::to_string(local_velocity_rotor0.x()) + ", " +
                             std::to_string(local_velocity_rotor0.y()) + ")."
                      << std::endl;
        }
    }

    // store induction factors for starting point of next time iteration
    element.induction_factor_axial = aa;
    element.induction_factor_tangential = ap;

    // return velocity in local
    return local_velocity_rotor;
}

void seahowl::aero::apply_tower_shadow_effect_on_wind(chrono::ChVector<double>& wind_velocity,
                                                      const chrono::ChVector<double>& position,
                                                      double blade_azimuth,
                                                      const seahowl::aero::TowerAero& tower_aero) {
    if (blade_azimuth > chrono::CH_C_PI / 2.0 && blade_azimuth < 3.0 * chrono::CH_C_PI / 2.0) {
        // get wind velocity in tower reference frame
        auto wind_velocity_tower0 = tower_aero.elements.back().properties.rotation.RotateBack(wind_velocity);
        // only take wind velocity perpendicular to tower axis
        auto wind_velocity_tower = wind_velocity_tower0;
        wind_velocity_tower.Set(0.0, wind_velocity.y(), wind_velocity_tower.z());

        // project element coordinates to tower reference frame
        auto& tower_top = tower_aero.elements.back();
        auto& tower_top_coords = tower_aero.elements.back().properties.coordinates;
        auto coordinates_projected =
            -tower_aero.elements.back().properties.rotation.RotateBack(position - tower_top_coords);

        // find tower radius
        auto tower_length =
            (tower_aero.reference_points.back().coordinates - tower_aero.reference_points.front().coordinates).Length();
        if (coordinates_projected.x() > 0.0) {
            std::vector<double> fractions{1.0 - (tower_length - coordinates_projected.x()) / tower_length};
            auto tower_radius =
                seahowl::core::get_discretized_points(fractions, tower_aero.reference_points)[0].diameter / 2.0;
            auto xx = coordinates_projected.z();
            auto xx2 = pow(xx, 2);
            auto yy = coordinates_projected.y();
            auto yy2 = pow(yy, 2);
            wind_velocity_tower *= (1.0 + (yy2 - xx2) / pow(yy2 + xx2, 2) * pow(tower_radius, 2));
            wind_velocity_tower.Set(wind_velocity_tower0.x(), wind_velocity_tower.y(), wind_velocity_tower.z());

            // correct wind velocity
            wind_velocity = tower_aero.elements.back().properties.rotation.Rotate(wind_velocity_tower);
        }
    }
}
