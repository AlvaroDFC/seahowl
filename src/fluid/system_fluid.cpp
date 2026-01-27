#include "seahowl/fluid/system_fluid.h"

#include <spdlog/spdlog.h>

using namespace seahowl::fluid;

void SystemFluid::add(std::shared_ptr<TurbineFluid> turbine) {
    if (std::find(turbines.begin(), turbines.end(), turbine) == turbines.end()) {
        turbines.push_back(turbine);
    } else
        spdlog::warn("Turbine fluid already exists in the system, not adding again.");
}

void SystemFluid::add(std::shared_ptr<seahowl::fluid::ComponentFluid> component) {
    if (std::find(components.begin(), components.end(), component) == components.end()) {
        components.push_back(component);
    } else
        spdlog::warn("Component fluid already exists in the system, not adding again.");
}

void SystemFluid::build() {
    // build all turbines
    for (auto& turbine : turbines) {
        turbine->build();
    }
    // build all extra components
    for (auto& component : components) {
        component->build();
    }
}

void SystemFluid::compute_env_loads(const env::EnvModel& env_model, double time) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        turbine->compute_env_loads(env_model, time);
    }
    for (auto& component : components) {
        // compute forces from fluid model
        component->compute_env_loads(env_model, time);
    }
}
