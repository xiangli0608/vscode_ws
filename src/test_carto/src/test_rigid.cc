#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
using namespace std;
using namespace Eigen;


class Rigid2 {
 public:
  using Vector = Eigen::Matrix<double, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<double>;
  Vector translation_;
  Rotation2D rotation_;
  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}

  Rigid2 inverse() const
  {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    cout << "inverse() " << endl
         << translation << endl
         << rotation.angle() << endl;
    return Rigid2(translation, rotation);
  }
};


void operator*(const Rigid2& lhs,
                 const Rigid2& rhs) {

      cout << (lhs.rotation_ * rhs.translation_ + lhs.translation_) << endl;
      cout << (lhs.rotation_ * rhs.rotation_).angle() << endl;
}

Rigid2::Vector operator*(
    const Rigid2& rigid,
    const Rigid2::Vector& point) {
  return rigid.rotation_ * point + rigid.translation_;
}


void test_rigid()
{
  Rigid2 lhs(Eigen::Matrix<double, 2, 1>(3, 0), Eigen::Rotation2D<double>(0) );
  Rigid2 rhs(Eigen::Matrix<double, 2, 1>(1, 0), Eigen::Rotation2D<double>(0.0) );
  // lhs * rhs;
  // lhs.inverse();
  
  Rigid2 sensor_to_tracking(Eigen::Matrix<double, 2, 1>(1, 0), Eigen::Rotation2D<double>(0) );
  // sensor_to_tracking.inverse();
  // lhs * sensor_to_tracking.inverse();
}

int main()
{

  Rigid2 sensor_to_tracking(Eigen::Matrix<double, 2, 1>(-1, 0), Eigen::Rotation2D<double>(0.784) );
  Rigid2::Vector point(3, 4);
  cout << sensor_to_tracking * point << endl;

  return 0;
}