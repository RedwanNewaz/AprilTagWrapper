//
// Created by redwan on 7/21/21.
//
#include <boost/python.hpp>
#include "april_tag_wrapper/april_tag_wrapper.h"
using namespace boost::python;

BOOST_PYTHON_MODULE(apriltag)
{
    class_<april_tag_wrapper>("apriltag")
            .def("get_pose", &april_tag_wrapper::get_pose)
            .def("detect", &april_tag_wrapper::detect)
            ;
}