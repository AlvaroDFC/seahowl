#include "seahowl/core/floater.h"

// SEAHOWL headers
#include "seahowl/core/mooring.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/fluid/hydro/floater_hydro.h"
#include "seahowl/fluid/hydro/mooring_hydro.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <memory>
#include <vector>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::fluid::hydro;

Floater::Floater(std::shared_ptr<seahowl::elasto::FloaterElasto> elasto,
                 std::shared_ptr<seahowl::hydro::FloaterHydro> hydro)
    : Foundation(elasto, hydro),
      ComponentDynamic(elasto, hydro),
      elasto(*elasto),
      hydro(*hydro),
      mooring_system(std::make_unique<MooringSystem>(elasto->mooring_system, hydro->mooring_system)) {}

Floater::~Floater() = default;

void Floater::initialize_this(double time, double dt) {
    mooring_system->initialize(time, dt);

    elasto.initialize();
    hydro.initialize(time, dt);

    spdlog::info("Initialized floater of total mass {:.4}kg (with moorings).", elasto.get_mass());
}

void Floater::prestep(double time, double dt) {
    mooring_system->prestep(time, dt);

    elasto.body_main->reset_loads_internals();

    // set hydro forces if any
    elasto.body_main->accumulate_force(hydro.get_force_hydro(), false);
    elasto.body_main->accumulate_torque(hydro.get_torque_hydro(), false);
    elasto.body_main->set_added_mass_matrix(hydro.get_added_mass_matrix());
}

void Floater::poststep(double time, double dt) {
    mooring_system->poststep(time, dt);
}

void Floater::apply_env_model(seahowl::env::EnvModel& env_model, double time) {
    mooring_system->apply_env_model(env_model, time);
}

void Floater::apply_soil_model(seahowl::env::EnvModel& env_model, double time) {
    mooring_system->apply_soil_model(env_model, time);
}

void Floater::build() {
    // build elasto
    elasto.build();
}
