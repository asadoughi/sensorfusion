#include <boost/python.hpp>
#include <Python.h>

#include "Fusion.h"

using namespace boost::python;
using namespace android;


struct vec4_to_list {
  static PyObject* convert(vec<float, 4ul> const& v) {
    list *l = new list();
    for (size_t i = 0; i < 4; i++) l->append(v[i]);
    return l->ptr();
  }
};


struct vec3_to_list {
  static PyObject* convert(vec<float, 3ul> const& v) {
    list *l = new list();
    for (size_t i = 0; i < 3; i++) l->append(v[i]);
    return l->ptr();
  }
};


struct mat33_to_list {
  static PyObject* convert(mat<float, 3ul, 3ul> const& matrix) {
    list *m = new list();
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
	m->append(matrix[i][j]);
      }
    }
    return m->ptr();
  }
};

struct pylist_converter {
  template <typename Container>
  pylist_converter&
  from_python()
  {
    converter::registry::push_back(&pylist_converter::convertible,
				   &pylist_converter::construct<Container>,
				   type_id<Container>());
    return *this;
  }

  static void* convertible(PyObject* object) {
    return PyObject_GetIter(object) ? object : NULL;
  }

  template <typename Container>
  static void construct(PyObject *object,
			converter::rvalue_from_python_stage1_data* data)
  {
    handle<> handle(borrowed(object));
    typedef converter::rvalue_from_python_storage<Container> storage_type;
    void *storage = reinterpret_cast<storage_type*>(data)->storage.bytes;
    data->convertible = new (storage) Container();
    for (size_t i = 0; i < PyList_Size(object); i++) {
      ((float *)data->convertible)[i] = PyFloat_AsDouble(PyList_GetItem(object, i));
    }
  }
};


BOOST_PYTHON_MODULE(glue)
{
  to_python_converter<vec<float, 4ul>, vec4_to_list>();
  to_python_converter<vec<float, 3ul>, vec3_to_list>();
  to_python_converter<mat<float, 3ul, 3ul>, mat33_to_list>();
  
  pylist_converter()
    .from_python<vec<float, 3ul> >()
    ;

  class_<Fusion>("Fusion")
    .def("getAttitude", &Fusion::getAttitude)
    .def("getBias", &Fusion::getBias)
    .def("getRotationMatrix", &Fusion::getRotationMatrix)
    .def("hasEstimate", &Fusion::hasEstimate)
    .def("handleAcc", &Fusion::handleAcc)
    .def("handleMag", &Fusion::handleMag)
    .def("handleGyro", &Fusion::handleGyro)
    ;
}
