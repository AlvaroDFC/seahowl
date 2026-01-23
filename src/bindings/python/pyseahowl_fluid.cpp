// pybind11 headers
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// SEAHOWL headers
#include <seahowl/fluid.h>

namespace py = pybind11;

void initialize_pyseahowl_aero(py::module& m_fluid);
void initialize_pyseahowl_hydro(py::module& m_fluid);

void initialize_pyseahowl_fluid(py::module& m) {
    // submodule
    auto m_fluid = m.def_submodule("fluid", "Fluid submodule.");

    // fluid/component_fluid.h
    py::class_<seahowl::fluid::ComponentFluid, std::shared_ptr<seahowl::fluid::ComponentFluid>>(m_fluid,
                                                                                                "ComponentFluid")
        .def("build", &seahowl::fluid::ComponentFluid::build)
        .def("compute_env_loads", &seahowl::fluid::ComponentFluid::compute_env_loads)
        .def("setup_environment", &seahowl::fluid::ComponentFluid::setup_environment);

    // fluid/system_fluid.h
    py::class_<seahowl::fluid::SystemFluid, std::shared_ptr<seahowl::fluid::SystemFluid>,
               seahowl::fluid::ComponentFluid>(m_fluid, "SystemFluid")
        .def(py::init<>())
        .def("add",
             static_cast<void (seahowl::fluid::SystemFluid::*)(std::shared_ptr<seahowl::fluid::TurbineFluid> turbine)>(
                 &seahowl::fluid::SystemFluid::add))
        .def("add", static_cast<void (seahowl::fluid::SystemFluid::*)(
                        std::shared_ptr<seahowl::fluid::ComponentFluid> component)>(&seahowl::fluid::SystemFluid::add))
        .def_readonly("turbines", &seahowl::fluid::SystemFluid::turbines)
        .def_readonly("components", &seahowl::fluid::SystemFluid::components);

    // fluid/turbine_fluid.h
    py::class_<seahowl::fluid::TurbineFluid, std::shared_ptr<seahowl::fluid::TurbineFluid>,
               seahowl::fluid::ComponentFluid>(m_fluid, "TurbineFluid")
        .def(py::init<>())
        .def_readonly("rna", &seahowl::fluid::TurbineFluid::rna)
        .def_readonly("tower", &seahowl::fluid::TurbineFluid::tower)
        .def_readonly("foundation", &seahowl::fluid::TurbineFluid::foundation);

    // hydro/foundation_fluid.h
    py::class_<seahowl::hydro::FoundationFluid, std::shared_ptr<seahowl::hydro::FoundationFluid>,
               seahowl::fluid::ComponentFluid>(m_fluid, "FoundationFluid");

    // aero submodule
    initialize_pyseahowl_aero(m_fluid);

    // hydro submodule
    initialize_pyseahowl_hydro(m_fluid);
}
