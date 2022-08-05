#pragma once

#include <vector>
#include <memory>

namespace seahowl {
namespace elasto {
class BladeElasto;
class TowerElasto;  ///@todo move out of rotor
}
}  // namespace seahowl


#include <chrono/core/ChVector.h>

namespace chrono {
class ChBody;
class ChLinkMateFix;
class ChLinkRevolute;
class ChSystemSMC;

}  // namespace chrono

namespace seahowl {
namespace elasto {

/**@brief Hub properties 


\image html NREL_ad_driver_geom.png "source image: NREL/Openfast" width=300cm
*/
struct HubProperties {
    double center_of_mass = 0.0; ///< COG Center of giration
    double mass = 0.0; ///< Total mass of the hub
    double inertia = 0.0; ///< Coefficient of inertia
    double overhang = 0.0; ///< Overhang 
    double radius = 0.0; ///< Radius of the hub ///@todo define better what is radius
};

/**@brief Nacelle properties */
struct NacelleProperties {
    chrono::ChVector<double> center_of_mass{0.0, 0.0, 0.0};  ///<@TODO Initialize in constructor
    double mass = 0.0; ///< Mass of the Nacelle (without bearing)
    double inertia = 0.0; ///< Coefficient of inertia
    double yaw_bearing_mass = 0.0; ///< Bearing mass
};

/**@brief Shaft properties */
struct ShaftProperties {
    double tilt = 0.0; ///< Tilt angle
    double distance_from_towertop = 0.0; ///< Distance from Tower top reference point
};

/**@brief Rotor properties */
class RotorElasto {
  public:
    std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades;
    std::vector<double> blade_precones;
    // bodies
    std::shared_ptr<chrono::ChBody> body_hub;
    std::shared_ptr<chrono::ChBody> body_shaft;
    std::shared_ptr<chrono::ChBody> body_nacelle;
    std::shared_ptr<chrono::ChBody> body_yaw_bearing;
    // links
    std::vector<std::shared_ptr<chrono::ChLinkMateFix>> links_blades;
    std::shared_ptr<chrono::ChLinkRevolute> link_shaft_hub;
    std::shared_ptr<chrono::ChLinkMateFix> link_shaft_nacelle;
    std::shared_ptr<chrono::ChLinkMateFix> link_shaft_yaw_bearing;
    std::shared_ptr<chrono::ChLinkMateFix> link_towertop_yaw_bearing;
    // properties
    ShaftProperties shaft;
    NacelleProperties nacelle;
    HubProperties hub;

    RotorElasto();

    void build(chrono::ChSystemSMC& system, std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades);
    void link_tower(TowerElasto& tower, chrono::ChSystemSMC& system);
    void rotate(double angle, chrono::ChVector<double> axis);
    void translate(chrono::ChVector<double> translation_vector);
    double get_mass();
    void apply_collective_pitch_increment(double pitch_increment);
    double get_rpm();
    double get_torque();
};

}  // namespace elasto
}  // namespace seahowl