#include "rotor.h"
#include "chrono/physics/ChBodyEasy.h"

Rotor::Rotor() {}

void Rotor::build(ChSystemSMC& system, std::vector<std::shared_ptr<BladeElasto>> blades) {
    this->blades = blades;

    // hub
    body_hub = chrono_types::make_shared<ChBodyEasyBox>(2.0, 2.0, 4.0, 0, true, false);
    system.Add(body_hub);
    // move hub along X for overhang and COG offset, and along Z for distance from towertop
    body_hub->SetPos(ChVector<double>(hub.overhang + hub.center_of_mass, 0.0, shaft.distance_from_towertop));
    // local Z axis along global X axis + shaft tilt along global Y axis
    auto tilt_hub = Q_from_AngAxis(shaft.tilt, -VECT_Y);
    body_hub->SetRot(tilt_hub * Q_from_AngAxis(CH_C_PI / 2.0, VECT_Y));
    body_hub->SetPos(tilt_hub.Rotate(body_hub->GetPos()));
    // mass and inertia
    body_hub->SetMass(hub.mass);
    body_hub->SetInertiaXX(ChVector<double>(0., 0., hub.inertia));

    // shaft
    body_shaft = chrono_types::make_shared<ChBodyEasyBox>(2.0, 2.0, 4.0, 0, true, false);
    system.Add(body_shaft);
    // move end of shaft at yaw axis of nacelle
    body_shaft->SetPos(ChVector<double>(0.0, 0.0, shaft.distance_from_towertop));
    // align rotation
    body_shaft->SetRot(body_hub->GetRot());
    // massless body
    body_shaft->SetMass(0.0);
    // link hub to shaft
    link_shaft_hub = chrono_types::make_shared<ChLinkRevolute>();
    system.Add(link_shaft_hub);
    link_shaft_hub->Initialize(body_hub, body_shaft, body_shaft->GetAssetsFrame());

    // nacelle
    body_nacelle = chrono_types::make_shared<ChBodyEasyBox>(2.0, 2.0, 4.0, 0, true, false);
    system.Add(body_nacelle);
    body_nacelle->SetPos(nacelle.center_of_mass);
    // mass and inertia
    body_nacelle->SetMass(nacelle.mass);
    // TODO: change to full 3x3 inertia matrix
    body_nacelle->SetInertiaXX(ChVector<double>(0.0, 0.0, nacelle.inertia));
    // link nacelle body to shaft body
    link_shaft_nacelle = chrono_types::make_shared<ChLinkMateFix>();
    system.Add(link_shaft_nacelle);
    link_shaft_nacelle->Initialize(body_nacelle, body_shaft);

    // yaw bearing
    body_yaw_bearing = chrono_types::make_shared<ChBodyEasyBox>(2.0, 2.0, 4.0, 0, true, false);
    system.Add(body_yaw_bearing);
    body_yaw_bearing->SetPos(ChVector<double>(0.0, 0.0, 0.0));
    body_yaw_bearing->SetMass(nacelle.yaw_bearing_mass);
    // link yaw bearing body to shaft body
    // link_shaft_yaw_bearing = chrono_types::make_shared<ChLinkRevolute>();
    link_shaft_yaw_bearing = chrono_types::make_shared<ChLinkMateFix>();
    system.Add(link_shaft_yaw_bearing);
    link_shaft_yaw_bearing->Initialize(body_shaft, body_yaw_bearing, body_yaw_bearing->GetAssetsFrame());

    // blades
    links_blades.clear();
    int nblades = blades.size();
    for (int ii = 0; ii < nblades; ii++) {
        auto blade = blades[ii];
        double precone = blade_precones[ii];

        // rotations + translations
        // blade root node is assumed to be originally at (0,0,0) and using IEC standard for coordinate system
        // apply precone
        blade->rotate(precone, VECT_Y);  // Y is the edge-wise axis for blade (IEC standard)
        double angle = ii * CH_C_2PI / nblades;
        // offset blade from hub apex and add overhang
        blade->translate(ChVector<double>(hub.overhang, 0.0, hub.radius));
        // rotate blade around hub
        blade->rotate(angle, VECT_X);  // X is the axis pointing towards nacelle for blade (IEC standard)
        // offset with distance from towertop
        blade->translate(ChVector<double>(0.0, 0.0, shaft.distance_from_towertop));
        // apply shaft tilt to blades
        blade->rotate(shaft.tilt, -VECT_Y);

        // link root node of blade to rotor center
        auto link_hub_blade = chrono_types::make_shared<ChLinkMateFix>();
        system.Add(link_hub_blade);
        link_hub_blade->Initialize(blade->nodes[0], body_hub);
        links_blades.push_back(link_hub_blade);
    }
}

void Rotor::link_tower(Tower& tower, ChSystemSMC& system) {
    auto towertop_node = tower.nodes[tower.nodes.size() - 1];
    // translate RNA center of origin to towertop
    this->translate(towertop_node->GetPos());
    // link yaw bearing body to towertop
    link_towertop_yaw_bearing = chrono_types::make_shared<ChLinkMateFix>();
    system.Add(link_towertop_yaw_bearing);
    link_towertop_yaw_bearing->Initialize(towertop_node, body_yaw_bearing);
}

void Rotor::rotate(double angle, ChVector<double> axis) {
    // blades
    for (int ii = 0; ii < blades.size(); ii++) {
        blades[ii]->rotate(angle, axis);
    }
    auto rotation = Q_from_AngAxis(angle, axis);
    // hub
    auto new_position_hub = rotation.Rotate(body_hub->GetPos());
    auto new_rotation_hub = (rotation * body_hub->GetRot()).GetNormalized();
    body_hub->SetPos(new_position_hub);
    body_hub->SetRot(new_rotation_hub);
    // shaft
    auto new_position_shaft = rotation.Rotate(body_shaft->GetPos());
    auto new_rotation_shaft = (rotation * body_shaft->GetRot()).GetNormalized();
    body_shaft->SetPos(new_position_shaft);
    body_shaft->SetRot(new_rotation_shaft);
    // nacelle
    auto new_position_nacelle = rotation.Rotate(body_nacelle->GetPos());
    auto new_rotation_nacelle = (rotation * body_nacelle->GetRot()).GetNormalized();
    body_nacelle->SetPos(new_position_nacelle);
    body_nacelle->SetRot(new_rotation_nacelle);
    // yaw bearing
    auto new_position_yaw_bearing = rotation.Rotate(body_yaw_bearing->GetPos());
    auto new_rotation_yaw_bearing = (rotation * body_yaw_bearing->GetRot()).GetNormalized();
    body_yaw_bearing->SetPos(new_position_yaw_bearing);
    body_yaw_bearing->SetRot(new_rotation_yaw_bearing);
}

void Rotor::translate(ChVector<double> translation_vector) {
    // blades
    for (int ii = 0; ii < blades.size(); ii++) {
        blades[ii]->translate(translation_vector);
    }
    // hub
    body_hub->SetPos(body_hub->GetPos() + translation_vector);
    // shaft
    body_shaft->SetPos(body_shaft->GetPos() + translation_vector);
    // nacelle
    body_nacelle->SetPos(body_nacelle->GetPos() + translation_vector);
    // yaw_bearing
    body_yaw_bearing->SetPos(body_yaw_bearing->GetPos() + translation_vector);
}

double Rotor::get_mass() {
    double total_mass = 0.0;
    // blades
    for (int ii = 0; ii < blades.size(); ii++) {
        total_mass += blades[ii]->get_mass();
    }
    // hub
    total_mass += body_hub->GetMass();
    // shaft
    total_mass += body_shaft->GetMass();
    // nacelle
    total_mass += body_nacelle->GetMass();
    // yaw_bearing
    total_mass += body_yaw_bearing->GetMass();
    return total_mass;
}

void Rotor::apply_collective_pitch_increment(double pitch_increment) {
    for (int ii = 0; ii < blades.size(); ii++) {
        // apply pitch on blade
        auto blade = blades[ii];
        blade->apply_pitch_increment(pitch_increment);
        // update blade-hub constraint
        auto link = links_blades[ii];
        link->Initialize(blade->nodes.front(), body_hub);
    }
}
