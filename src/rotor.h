#ifndef ROTOR_H_
#define ROTOR_H_

#include "blade.h"
#include "tower.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkRevolute.h"

using namespace chrono;

struct HubProperties {
    double center_of_mass = 0.0;
    double mass = 0.0;
    double inertia = 0.0;
    double overhang = 0.0;
    double radius = 0.0;
};

struct NacelleProperties {
    ChVector<double> center_of_mass = ChVector<double>(0.0, 0.0, 0.0);
    double mass = 0.0;
    double inertia = 0.0;
    double yaw_bearing_mass = 0.0;
};

struct ShaftProperties {
    double tilt = 0.0;
    double distance_from_towertop = 0.0;
};

class Rotor {
  public:
    std::vector<std::shared_ptr<Blade>> blades;
    std::vector<double> blade_precones;
    // bodies
    std::shared_ptr<ChBody> body_hub;
    std::shared_ptr<ChBody> body_shaft;
    std::shared_ptr<ChBody> body_nacelle;
    std::shared_ptr<ChBody> body_yaw_bearing;
    // links
    std::shared_ptr<ChLinkRevolute> link_shaft_hub;
    std::shared_ptr<ChLinkMateFix> link_shaft_nacelle;
    std::shared_ptr<ChLinkMateFix> link_shaft_yaw_bearing;
    std::shared_ptr<ChLinkMateFix> link_towertop_yaw_bearing;
    // properties
    ShaftProperties shaft;
    NacelleProperties nacelle;
    HubProperties hub;

    Rotor();

    void build(ChSystemSMC& system, std::vector<std::shared_ptr<Blade>> blades);
    void link_tower(Tower& tower, ChSystemSMC& system);
    void rotate(double angle, ChVector<double> axis);
    void translate(ChVector<double> translation_vector);
    double get_mass();
};

#endif  // ROTOR_H_
