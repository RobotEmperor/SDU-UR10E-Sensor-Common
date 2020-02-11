#include "ur10e_force_torque_sensor/ur10e_force_torque_sensor.h"
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

int add(int i, int j)
{
  return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(ur10e_force_torque_sensor_py, m)
{
m.doc() = R"pbdoc(
        UR10e Force torque sensor filtering and the signal process library.
        -----------------------
        .. currentmodule:: ur10e_force_torque_sensor_py
        .. autosummary::
           :toctree: _generate
           parse_init_data
           initialize
           offset_init
           signal_processing
           collision_detection
    )pbdoc";


py::class_<Ur10eFTsensor>(m, "Ur10eFTsensor")
  .def(py::init<>())
  .def("parse_init_data", &Ur10eFTsensor::parse_init_data, R"pbdoc(To load sensor filter gains, you can insert the path of init_data.yaml.)pbdoc")
  .def("initialize", &Ur10eFTsensor::initialize, R"pbdoc(Variables are initialized in this function.)pbdoc")
  .def("offset_init", &Ur10eFTsensor::offset_init, R"pbdoc(This function is to get offest value by sampling raw data of FT sensor for a certain period of time.)pbdoc")
  .def("signal_processing", &Ur10eFTsensor::signal_processing, R"pbdoc(Signal processing using a kalman filter.)pbdoc")
  .def("collision_detection", &Ur10eFTsensor::collision_detection_processing, R"pbdoc(This function can detect collision from raw force torque data by using CUSUM method. It returns int value 1 and -1 when collision is detected. Value 0 is default and non-contact)pbdoc")
  .def("get_ft_filtered_data", &Ur10eFTsensor::get_ft_filtered_data, R"pbdoc(Returns filtered force torque data.)pbdoc")
  .def("__repr__", [](const Ur10eFTsensor &a) { return "<Ur10eFTsensor>"; }); 

#ifdef VERSION_INFO
m.attr("__version__") = VERSION_INFO;
#else
m.attr("__version__") = "dev";
#endif
}
