#include "seahowl/core/component.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <typeinfo>

using namespace seahowl::core;

void ComponentDynamic::initialize(double time, double dt) {
    spdlog::debug("Initialization of component: {}.", std::string(typeid(*this).name()));
    if (is_initialized) {
        throw std::runtime_error("Component already initialized: " + std::string(typeid(*this).name()) + ".");
    }

    initialize_this(time, dt);  // component-specific initialization
    is_initialized = true;

    spdlog::debug("Finished initialization of component: {}.", std::string(typeid(*this).name()));
}
