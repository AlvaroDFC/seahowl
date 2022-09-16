#pragma once
#include <seahowl/core/turbine.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/servo/controller.h>
#include <seahowl/servo/controller_discon.h>
#include <chrono/physics/ChBody.h>

namespace seahowl {
namespace core {

class System {
  public:
    Turbine turbine;
    seahowl::aero::WindModel& wind_model;

    System(Turbine turbine, seahowl::aero::WindModel& wind_model);
    ~System();

    virtual void init(double time, double dt);
    virtual void prestep(double time, double dt);
    virtual void poststep(double time, double dt);
};

class SystemVariableTorque : public System {
  public:
    seahowl::servo::ControllerVariableTorque controller;
    SystemVariableTorque(Turbine turbine,
                         seahowl::aero::WindModel& wind_model,
                         seahowl::servo::ControllerVariableTorque controller);
    ~SystemVariableTorque();

    void init(double time, double dt);
    void prestep(double time, double dt);
    void poststep(double time, double dt);
};

class SystemDISCON : public System {
  public:
    seahowl::servo::ControllerDISCON controller;

    SystemDISCON(Turbine turbine, seahowl::aero::WindModel& wind_model, seahowl::servo::ControllerDISCON controller);
    ~SystemDISCON();

    virtual void init(double time, double dt);
    virtual void poststep(double time, double dt);
};
}  // namespace core
}  // namespace seahowl