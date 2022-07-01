#include "blade.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemSMC.h"

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
    std::vector<Blade*> blades;
    std::vector<double> blade_precones;
    std::shared_ptr<ChBody> body_hub_apex;
    std::shared_ptr<ChBody> body_shaft_hub;
    ShaftProperties shaft;
    NacelleProperties nacelle;
    HubProperties hub;

    Rotor();

    void build(ChSystemSMC& system, std::vector<Blade*> blades);
    void rotate(double angle, ChVector<double> axis);
    void translate(ChVector<double> translation_vector);
};
