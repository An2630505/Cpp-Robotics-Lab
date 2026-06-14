#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "control/mpc.h"

namespace py = pybind11;

PYBIND11_MODULE(pnc, m) {
    m.doc() = "PNC Robotics Lab — C++ algorithm library";

    py::class_<MPC>(m, "MPC")
        .def(py::init<>())
        .def("init", &MPC::Init,
             py::arg("A"), py::arg("B"), py::arg("C"),
             py::arg("Q"), py::arg("R"), py::arg("S"),
             py::arg("N"))
        .def("predict", &MPC::predict,
             py::arg("y_ref"), py::arg("x_obs"));
}
