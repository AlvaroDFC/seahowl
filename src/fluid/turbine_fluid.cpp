#include "seahowl/fluid/turbine_fluid.h"

// SEAHOWL headers
#include "seahowl/env/env_model.h"

using namespace seahowl::fluid;
using seahowl::env::EnvModel;

TurbineFluid::TurbineFluid()
    : rna(std::make_shared<aero::RotorNacelleAssemblyAero>()), tower(std::make_shared<aero::TowerAero>()) {}

void TurbineFluid::build() {
    rna->build();
    tower->build();
    if (foundation) {
        foundation->build();
    }
}

void TurbineAero::setup_environment(const env::EnvModel& env_model) {
    rna->setup_environment(env_model);
    tower->setup_environment(env_model);
    if (foundation) {
        foundation->setup_environment(env_model);
    }
}

void TurbineFluid::initialize(double time, double dt) {}

void TurbineFluid::compute_env_loads(const EnvModel& env_model, double time) {
    rna->compute_env_loads(env_model, time);
    tower->compute_env_loads(env_model, time);

    if (foundation) {
        foundation->compute_env_loads(env_model, time);
    }
}
