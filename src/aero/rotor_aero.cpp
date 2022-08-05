#include "seahowl/aero/rotor_aero.h"

void RotorAero::build(std::vector<std::shared_ptr<BladeAero>> blades) {
    this->blades = blades;

    // calculate rotor radius
    radius = 0.0;
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        radius += (blade->discretized_points.back().m_coordinates - hub_position).Length();
    }
    radius /= blades.size();

    // compute blade elements related values
    compute_distances_from_tip();
    compute_distances_from_hub();
    compute_radii();
    compute_chords_solidity();
}

void RotorAero::compute_chords_solidity() {
    auto nblades = blades.size();
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        for (int jj = 0; jj < blade->elements.size(); jj++) {
            auto& element = blade->elements[jj];
            auto radius = (element.properties.m_coordinates - hub_position).Length();
            element.swept_annulus = element.length * 2 * CH_C_PI * radius;
            element.chord_solidity = nblades * element.properties.chord / (2 * CH_C_PI * radius);
            // std::cout << element.swept_annulus << " " << element.chord_solidity << std::endl;
        }
    }
}

void RotorAero::compute_distances_from_hub() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_distances_from_hub(hub_position, hub_radius);
    }
}

void RotorAero::compute_distances_from_tip() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_distances_from_tip();
    }
}

void RotorAero::compute_radii() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_radii(hub_position);
    }
}

void RotorAero::compute_wind_loads_bemt(WindModel& wind_model, double time) {
    auto wind_velocity = wind_model.get_wind_velocity(hub_position, 0.0);
    auto local_velocity0_3d = hub_rotation.RotateBack(wind_velocity);

    double density = wind_model.get_density();
    for (int kk = 0; kk < 3; kk++) {
        auto& blade = blades[kk];
        for (int ii = 0; ii < blade->elements.size(); ii++) {
            auto& element = blade->elements[ii];
            auto& properties = element.properties;

            // get fluid relative velocity
            auto wind_velocity = wind_model.get_wind_velocity(properties.m_coordinates, time);
            auto global_velocity = wind_velocity - properties.velocity;
            // project in disc frame
            auto local_velocity_disc = hub_rotation.RotateBack(global_velocity);

            // get global/local directions
            // pointing from hub towards nacelle
            auto local_direction_normal = ChVector<double>(0.0, 0.0, 1.0);
            auto global_direction_normal = hub_rotation.Rotate(local_direction_normal);
            // pointing from hub to element position
            auto global_direction_hub2element = (properties.m_coordinates - hub_position).GetNormalized();
            // pointing in tangential direction
            auto global_direction_tangent = (global_direction_hub2element % global_direction_normal).GetNormalized();

            // uninduced local velocity (2D)
            // frame perpendicular to rotor disc
            // x: tangential velocity (coplanar with rotor disc)
            // y: normal velocity (normal to rotor disc, pointing from hub to nacelle)
            double local_velocity_normal = (global_velocity ^ global_direction_normal);
            double local_velocity_tangent = (global_velocity ^ global_direction_tangent);
            auto local_velocity0 = ChVector2<double>(local_velocity_tangent, local_velocity_normal);

            // get induced velocity (2D) from blade element
            auto local_velocity = element.get_induced_velocity_rotor(local_velocity0, blades.size());

            // get coefficients from angle of attack
            double phi = atan2(local_velocity.y(), -local_velocity.x());
            double alpha = phi - (element.pitch + element.properties.structural_twist);
            // check that alpha is still in range
            if (alpha < -CH_C_PI || alpha > CH_C_PI) {
                alpha = abs(std::fmod((alpha + 3 * CH_C_PI), 2 * CH_C_PI)) - CH_C_PI;
            }
            auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180 / CH_C_PI);

            // calculate drag and lift force
            double vel = local_velocity.Length();
            double chord = properties.chord;
            double length = element.length;
            double lift = 0.5 * density * vel * vel * chord * coefficients.lift * length;
            double drag = 0.5 * density * vel * vel * chord * coefficients.drag * length;

            // projected to rotor local frame
            double cx = lift * cos(phi) + drag * sin(phi);
            double cy = lift * sin(phi) - drag * cos(phi);

            // transform from local to global load
            auto lift_global = global_direction_tangent * cy;
            auto drag_global = global_direction_normal * cx;
            auto load_global = lift_global + drag_global;

            // store load in global frame
            blade->loads[ii] = load_global;
        }
    }
}
