#include "seahowl/env/seastate_adapter.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <fstream>
#include <spdlog/spdlog.h>
#include <filesystem>

using namespace seahowl::env;
namespace fs = std::filesystem;

/**
 * @brief SeaState module in OpenFAST
 */
extern "C" {

//
//
// DECLARE SEASTATE C API HERE
//
//
}

/**
 * @brief SeaState wrapping inferface
 */
struct seahowl::env::SeaStateLib {
    ~SeaStateLib();
};

SeaStateLib::~SeaStateLib() {}

SeaStateAdapter::SeaStateAdapter(std::string seastate_infile) {
    spdlog::info("Using SeaState.");
    this->seastate_infile = seastate_infile;
}

SeaStateAdapter::~SeaStateAdapter() {}

double SeaStateAdapter::get_water_level(const Vector3d& position, double time) const {
    // spdlog::warn("Node position ({}, {}, {})", position.x(), position.y(), position.z());
    // spdlog::warn("  -> returning zero water level from SeaStateAdapter (to implement).");
    return 0.0;
};

double SeaStateAdapter::get_density_this(const seahowl::Vector3d& position, double time) const {
    spdlog::warn("Node position ({}, {}, {})", position.x(), position.y(), position.z());
    spdlog::warn("  -> returning zero fluid density from SeaStateAdapter (to implement).");
    return 0.0;
}

seahowl::Vector3d SeaStateAdapter::get_velocity_this(const seahowl::Vector3d& position, double time) const {
    spdlog::warn("Node position ({}, {}, {})", position.x(), position.y(), position.z());
    spdlog::warn("  -> returning zero fluid velocity from SeaStateAdapter (to implement).");
    return seahowl::Vector3d(0.0, 0.0, 0.0);
}

seahowl::Vector3d SeaStateAdapter::get_acceleration_this(const seahowl::Vector3d& position, double time) const {
    spdlog::warn("Node position ({}, {}, {})", position.x(), position.y(), position.z());
    spdlog::warn("  -> returning zero fluid acceleration from SeaStateAdapter (to implement).");
    return seahowl::Vector3d(0.0, 0.0, 0.0);
}
