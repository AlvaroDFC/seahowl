#include "rotor.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkRevolute.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

Rotor::Rotor() {}

void Rotor::build(ChSystemSMC& system, std::vector<Blade*> blades) {
    // set hub apex at (0,0,0)
    // body_hub_apex = chrono_types::make_shared<ChBody>();
    // body_shaft_hub = chrono_types::make_shared<ChBody>();
    body_hub_apex = chrono_types::make_shared<ChBodyEasyBox>(2.0, 2.0, 2.0, 0, true, false);
    body_shaft_hub = chrono_types::make_shared<ChBodyEasyBox>(2.0, 2.0, 2.0, 0, true, false);
    system.Add(body_hub_apex);
    system.Add(body_shaft_hub);

    body_hub_apex->SetPos(ChVector<>(hub.overhang + hub.center_of_mass, 0.0, shaft.distance_from_towertop));
    // local Z axis along global X axis + shaft tilt along global Y axis
    auto tilt_hub = Q_from_AngAxis(shaft.tilt, -VECT_Y);
    body_hub_apex->SetRot(tilt_hub * Q_from_AngAxis(CH_C_PI / 2.0, VECT_Y));
    body_hub_apex->SetPos(tilt_hub.Rotate(body_hub_apex->GetPos()));

    // move end of shaft at hub apex
    body_shaft_hub->SetPos(ChVector(0.0, 0.0, shaft.distance_from_towertop));
    // align rotation
    body_shaft_hub->SetRot(body_hub_apex->GetRot());

    // link hub apex to shaft
    auto link_shaft_hub = chrono_types::make_shared<ChLinkRevolute>();
    system.Add(link_shaft_hub);
    link_shaft_hub->Initialize(body_hub_apex, body_shaft_hub, body_shaft_hub->GetAssetsFrame());

    int nblades = blades.size();
    for (int ii = 0; ii < nblades; ii++) {
        auto blade = blades[ii];
        double precone = blade_precones[ii];

        // rotations + translations
        // blade root node is assumed to be originally at (0,0,0) and using IEC standard for coordinate system
        // apply precone
        blade->rotate(precone, VECT_Y);  // Y is the edge-wise axis for blade (IEC standard)
        double angle = ii * CH_C_2PI / nblades;
        // offset blade from hub and add overhang
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
        link_hub_blade->Initialize(body_hub_apex, blade->nodes[0], body_hub_apex->GetAssetsFrame());
    }
}

void Rotor::rotate(double angle, ChVector<double> axis) {
    // blades
    for (int ii = 0; ii < blades.size(); ii++) {
        blades[ii]->rotate(angle, axis);
    }
    auto rotation = Q_from_AngAxis(angle, axis);
    // hub apex
    auto new_position_hub = rotation.Rotate(body_hub_apex->GetPos());
    auto new_rotation_hub = (rotation * body_hub_apex->GetRot()).GetNormalized();
    body_hub_apex->SetPos(new_position_hub);
    body_hub_apex->SetRot(new_rotation_hub);
    // shaft
    auto new_position_shaft = rotation.Rotate(body_shaft_hub->GetPos());
    auto new_rotation_shaft = (rotation * body_shaft_hub->GetRot()).GetNormalized();
    body_shaft_hub->SetPos(new_position_shaft);
    body_shaft_hub->SetRot(new_rotation_shaft);
}

void Rotor::translate(ChVector<double> translation_vector) {
    // blades
    for (int ii = 0; ii < blades.size(); ii++) {
        blades[ii]->translate(translation_vector);
    }
    // hub apex
    body_hub_apex->SetPos(body_hub_apex->GetPos() + translation_vector);
    // shaft
    body_shaft_hub->SetPos(body_shaft_hub->GetPos() + translation_vector);
}
