#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>
#include <boost/python.hpp>
#include <iostream>
using namespace std;
using namespace Eigen;
namespace py = boost::python;
//def logmap(m):
//    "http://en.wikipedia.org/wiki/Axis_angle#Log_map_from_SO.283.29_to_so.283.29"
//    theta = np.arccos((np.trace(m) - 1)/2)
//    return (1/(2*np.sin(theta))) * np.array([[m[2,1] - m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1]]])
//

Vector3d LogMap(const Matrix3d& m) {
  double cosarg = (m.trace() - 1)/2;
  cosarg = fmin(cosarg, 1);
  cosarg = fmax(cosarg, -1);
  double theta = acos( cosarg );
  if (theta==0) return Vector3d::Zero();
  else return theta*(1/(2*sin(theta))) * Vector3d(m(2,1) - m(1,2), m(0,2)-m(2,0), m(1,0)-m(0,1));
}

double RotReg(const Matrix3d& b, const Vector3d& rot_coeffs, double scale_coeff) {
  // regularize rotation using polar decomposition
  JacobiSVD<Matrix3d> svd(b.transpose(), ComputeFullU | ComputeFullV);
  Vector3d s = svd.singularValues();
  if (b.determinant() <= 0) return INFINITY;
  return LogMap(svd.matrixU() * svd.matrixV().transpose()).cwiseAbs().dot(rot_coeffs) + s.array().log().square().sum()*scale_coeff;
}

Matrix3d RotRegGrad(const Matrix3d& b, const Vector3d& rot_coeffs, double scale_coeff) {
  Matrix3d out;
  double y0 = RotReg(b, rot_coeffs, scale_coeff);
  Matrix3d xpert = b;
  double epsilon = 1e-5;
  for (int i=0; i < 3; ++i) {
    for (int j=0; j < 3; ++j) {
      xpert(i,j) = b(i,j) + epsilon;
      out(i,j) = (RotReg(xpert, rot_coeffs, scale_coeff) - y0)/epsilon;
      xpert(i,j) = b(i,j);
    }
  }
  return out;
}

Vector3d gRotCoeffs;
double gScaleCoeff;

void PySetCoeffs(py::object rot_coeffs, py::object scale_coeff) {
  gRotCoeffs = Vector3d(py::extract<double>(rot_coeffs[0]), py::extract<double>(rot_coeffs[1]), py::extract<double>(rot_coeffs[2]));
  gScaleCoeff = py::extract<double>(scale_coeff);
}

template <typename T>
T* getPointer(const py::object& arr) {
  long int i = py::extract<long int>(arr.attr("ctypes").attr("data"));
  T* p = (T*)i;
  return p;
}


double PyRotReg(const py::object& m ){
  const double* data = getPointer<double>(m);
  return RotReg( Map< const Matrix<double,3,3,RowMajor> >(data), gRotCoeffs, gScaleCoeff);
}

py::object PyRotRegGrad(const py::object& m) {
  static py::object np_mod = py::import("numpy");
  py::object out = np_mod.attr("empty")(py::make_tuple(3,3));
  const double* data = getPointer<double>(m);
  Matrix<double,3,3,RowMajor> g = RotRegGrad( Map< const Matrix<double,3,3,RowMajor> >(data), gRotCoeffs, gScaleCoeff);
  memcpy(getPointer<double>(out), g.data(), sizeof(double)*9);
  return out;
}

BOOST_PYTHON_MODULE(fastmath) {
  py::def("set_coeffs", &PySetCoeffs);
  py::def("rot_reg", &PyRotReg);
  py::def("rot_reg_grad", &PyRotRegGrad);
}
