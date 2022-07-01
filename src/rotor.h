#include "blade.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemSMC.h"

using namespace chrono;

class Rotor {
  public:
    std::vector<Blade*> blades;
    std::vector<double> blade_precones;
    std::vector<double> blade_offsets;
    std::shared_ptr<ChBody> body_hub_apex;
    std::shared_ptr<ChBody> body_shaft_hub;
    double shaft_tilt;

    Rotor(std::vector<Blade*> blades, std::vector<double> blade_offsets, std::vector<double> blade_precones);

    void make_rotor(ChSystemSMC& system);
    void rotate(double angle, ChVector<double> axis);
    void translate(ChVector<double> translation_vector);
};
