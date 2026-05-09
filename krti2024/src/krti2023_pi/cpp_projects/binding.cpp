// binding.cpp
#include <pybind11/pybind11.h>
#include <cppmult.hpp>

PYBIND11_MODULE(binding, m) {
    m.doc() = "pybind11 example plugin"; // Optional module docstring
    m.def("cppmult", &cppmult, "A function that multiplies an int and a float");
}
