#include "numpy_boost.hpp"
#include <boost/python.hpp>
#include <iostream>
#include <string>
using namespace std;
namespace py = boost::python;
typedef numpy_boost<float, 1> npVectorf;
typedef numpy_boost<float, 2> npMatrixf;

py::object main_module;
py::object main_namespace;
void setup_python() {
    Py_Initialize();
    _import_array(); // no idea what this does but I got the idea here http://www.mail-archive.com/numpy-discussion@scipy.org/msg09372.html

    main_module = py::import("__main__");
    main_namespace = main_module.attr("__dict__");

    py::exec("import sys", main_namespace);
    py::exec("sys.argv = ['use_pr2_from_cpp']", main_namespace); // otherwise sys.argv is none and ros imports give errors
}

template <class npArrayType>
py::object toObject(npArrayType array) {
    PyObject* ptr = array.py_ptr();
    py::object arrayobj(py::handle<>(py::borrowed(ptr)));
    return arrayobj;
}


int main(int argc, char *argv[]) {
  try {
    setup_python();
    
    py::object pr2_module = py::import("pr2");
    
    int dims[1] = {7};
    npVectorf array(dims);
    for (int i=0; i < dims[0]; i++) array[i] = 0;
    
    py::object arrayobj = toObject(array);
    
    py::object pr2 = pr2_module.attr("PR2")();

    cout << "press enter to continue" << endl;
    cin.get();
    pr2.attr("rarm").attr("goto_joint_positions")(arrayobj);
    pr2.attr("rarm").attr("goto_posture")("up");
    cout << "press enter to continue" << endl;
    cin.get();

    
  }
  catch(...){
    PyErr_Print();
    PyErr_Clear();    
    return 1;
  }  
  
}
