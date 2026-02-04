#include "seahowl/io/viz_insitu.h"

// SEAHOWL headers
#include "seahowl/core/system.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl::io;

VisualizationInSitu::VisualizationInSitu() {
    spdlog::info("No concrete in situ visualization application selected.");
}

void VisualizationInSitu::initialize_elasto(seahowl::elasto::SystemElasto& system) {}

void VisualizationInSitu::initialize(seahowl::core::System& system) {}

void VisualizationInSitu::draw() {}
