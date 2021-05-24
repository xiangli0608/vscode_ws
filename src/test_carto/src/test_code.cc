#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "std_msgs/Int8.h"

#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

void test()
{
  double x0 = 1, y0 = 0, x1 = 1.6, y1 = 0;

  bool first_point_flag = true;
  double current_point_time = 0;
  double current_point_x = 0, current_point_y = 0, current_point_z = 1.0;

  Eigen::Affine3f transStartInverse, transFinal, transBt;

  float rotXCur = 0, rotYCur = 0, rotZCur = 0;

  float posXCur = 1, posYCur = 0, posZCur = 0;

  // 点云中的第一个点 求 transStartInverse，之后在这帧数据畸变过程中不再改变
  if (first_point_flag == true)
  {
    transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                                rotXCur, rotYCur, rotZCur))
                            .inverse();
    first_point_flag = false;
  }
  std::cout << "startInv: " << std::endl
            << transStartInverse.matrix() << std::endl;

  posXCur = 1.2;
  posYCur = 0;
  posZCur = 0;

  // transform points to start
  transFinal = pcl::getTransformation(posXCur, posYCur, posZCur,
                                      rotXCur, rotYCur, rotZCur);

  std::cout << "transFinal: " << std::endl
            << transFinal.matrix() << std::endl;

  // 该点相对第一个点的变换矩阵　=　第一个点在lidar世界坐标系下的变换矩阵的逆 × 当前点时lidar世界坐标系下变换矩阵
  transBt = transStartInverse * transFinal;

  // transBt = transBt.inverse();

  std::cout << "transBt: " << std::endl
            << transBt.matrix() << std::endl;

  // 根据lidar位姿变换，修正点云位置
  double x, y, z;
  x = transBt(0, 0) * x1 + transBt(0, 1) * y1 + transBt(0, 2) * current_point_z + transBt(0, 3);
  y = transBt(1, 0) * x1 + transBt(1, 1) * y1 + transBt(1, 2) * current_point_z + transBt(1, 3);
  z = transBt(2, 0) * x1 + transBt(2, 1) * y1 + transBt(2, 2) * current_point_z + transBt(2, 3);
  std::cout << "x: " << x << " y " << y << " z " << z << std::endl;
}

void testodom()
{
  double roll = 0, pitch = 0, yaw = 0;

  Eigen::Affine3f transBegin = pcl::getTransformation(
      1,
      0,
      0,
      roll, pitch, yaw);

  std::cout << "transBegin: " << std::endl
            << transBegin.matrix() << std::endl;

  Eigen::Affine3f transEnd = pcl::getTransformation(
      1.3,
      0,
      0,
      roll, pitch, yaw);

  std::cout << "transEnd: " << std::endl
            << transEnd.matrix() << std::endl;

  // 求得这之间的变换
  Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

  std::cout << "transBt: " << std::endl
            << transBt.matrix() << std::endl;

  // 通过　transBt　获取　odomIncreX等，一帧点云数据时间的odom变化量
  float odom_incre_x_, odom_incre_y_, odom_incre_z_;
  float rollIncre, pitchIncre, yawIncre;
  pcl::getTranslationAndEulerAngles(transBt,
                                    odom_incre_x_, odom_incre_y_, odom_incre_z_,
                                    rollIncre, pitchIncre, yawIncre);
  std::cout << "x: " << odom_incre_x_ << " y " << odom_incre_y_ << " z " << odom_incre_z_ << std::endl;
}

void test2()
{

  std::vector<unsigned char> vec(4, 0);
  vec[0] = 0x37;
  vec[1] = 0x02;
  std_msgs::Int8 id, action;
  id.data = 5;
  action.data = 0;
  vec[2] = id.data;
  vec[3] = action.data;
  std::cout << "lx" << std::endl;
  // ROS_INFO_STREAM( "id " << vec[2] << " action " << vec[3] );
}

void test3()
{
  Eigen::Vector3d start(0.0, 1.0, 2.0);
}

void test_lambda()
{
  std::vector<int> nums = {1, 5, 3, 4, 2, -1, 10};
  // std::sort(nums.begin(), nums.end(), [a , b](int a, int b) mutable throw() -> bool {
  //     // lambda 表达式函数体，在这里做到了将输入数组升序排列
  //     return (std::abs(a) < std::abs(b));
  // });
  // for (int i : nums)
  //   std::cout << i << " ";

  int a = 1, b = 2;
  const int c = [&a, b]() -> int {
    std::cout << "a: " << a << " b: " << b << std::endl;
    a = 3;
    return a + b;
  }();

  std::cout << "c: " << c << std::endl;

  std::cout << std::endl;
}

void test_lower_bound()
{
  double a[] = {1, 2, 3, 4, 5, 7, 8, 9};
  auto it = lower_bound(a, a + 8, 2.3, [](const double a, const double b) {
    return a < b;
  });
  cout << "lower_bound: " << *it << endl;
}

void test_quaterniond()
{
  double w = 0.991, x = 0.0, y = 0.0, z = 0.131;
  Eigen::Quaterniond last_orientation(w, x, y, z);
  Eigen::Vector3d euler_angles_last = last_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "last pitch yaw roll = " << euler_angles_last.transpose() * 180 / M_PI << endl;

  w = 0.974, x = 0.0, y = 0.0, z = 0.225;
  Eigen::Quaterniond new_orientation(w, x, y, z);
  Eigen::Vector3d euler_angles_new = new_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "new pitch yaw roll = " << euler_angles_new.transpose() * 180 / M_PI << endl;

  Eigen::Quaterniond rotate_delta = last_orientation.inverse() * new_orientation;
  Eigen::Vector3d euler_angles_delta = rotate_delta.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "delta pitch yaw roll = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

  new_orientation = last_orientation * rotate_delta;
  euler_angles_new = new_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "new 2 pitch yaw roll = " << euler_angles_new.transpose() * 180 / M_PI << endl;

}

template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1> &angle_axis)
{
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8; // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle)
  {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}

void test_imu_tracker()
{
  cout << "\n------------ orientation_  -------------" << endl;
  // 初始姿态
  Eigen::Quaterniond orientation_(Eigen::Quaterniond::Identity());
  cout << "orientation_: "
       << orientation_.w() << " "
       << orientation_.x() << " "
       << orientation_.y() << " "
       << orientation_.z() << endl;

  // 角速度
  Eigen::Vector3d imu_angular_velocity_(0, 0.349, 0);
  // 根据角速度预测出来的姿态变化
  Eigen::Quaterniond rotation =
      AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * 1));
  Eigen::Vector3d euler_angles_delta;
  euler_angles_delta = rotation.matrix().eulerAngles(0, 1, 2);
  cout << "\nrotation matrix to roll pitch yaw (deg) = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

  // 预测出的新的姿态
  orientation_ = (orientation_ * rotation).normalized();
  cout << "orientation_ after rotate: "
       << orientation_.w() << " "
       << orientation_.x() << " "
       << orientation_.y() << " "
       << orientation_.z() << endl;
  euler_angles_delta = orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "orientation_ after rotate roll pitch yaw (deg) = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

  cout << "\n------------ conjugate -------------" << endl;

  Eigen::Quaterniond conjugate = orientation_.conjugate();
  cout << "orientation_ after rotate conjugate: "
       << conjugate.w() << " "
       << conjugate.x() << " "
       << conjugate.y() << " "
       << conjugate.z() << endl;
  euler_angles_delta = conjugate.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "conjugate roll pitch yaw (deg) = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

  Eigen::Vector3d tmp_vector1 = orientation_ * Eigen::Vector3d::UnitZ();
  cout << "\ntmp_vector1 x y z = " << tmp_vector1.transpose() << endl;
  Eigen::Vector3d tmp_vector2 = orientation_.conjugate() * Eigen::Vector3d::UnitZ();
  cout << "tmp_vector2 x y z = " << tmp_vector2.transpose() << endl;

  cout << "\n------------ gravity_vector -------------" << endl;
  // 重力向量初值
  Eigen::Vector3d gravity_vector_ = Eigen::Vector3d::UnitZ();
  cout << "gravity_ origin x y z = " << gravity_vector_.transpose() << endl;
  // 由于机器人发生旋转, 预测出的新的重力向量
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  cout << "gravity_ after rotate x y z = " << gravity_vector_.transpose() << endl;

  cout << "\n------------ update gravity_vector -------------" << endl;
  // 根据线性加速度进行重力的校准
  Eigen::Vector3d imu_linear_acceleration(0, 0, 9.8);
  const double alpha = 0.5;
  // 需要验证结果,就把下面这句话注释掉
  gravity_vector_ = (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  cout << "gravity_ new x y z = " << gravity_vector_.transpose() << endl;

  cout << "\n------------ rotation_new -------------" << endl;
  // 得出预测的和校准后的旋转差值
  const Eigen::Quaterniond rotation_new = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  cout << "rotation_new : "
       << rotation_new.w() << " "
       << rotation_new.x() << " "
       << rotation_new.y() << " "
       << rotation_new.z() << endl;
  euler_angles_delta = rotation_new.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "rotation_new roll pitch yaw (deg) = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

  cout << "\n------------ orientation_new -------------" << endl;
  // 对姿态进行校准
  orientation_ = (orientation_ * rotation_new).normalized();
  cout << "orientation_ after update: "
       << orientation_.w() << " "
       << orientation_.x() << " "
       << orientation_.y() << " "
       << orientation_.z() << endl;
  euler_angles_delta = orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << "orientation_ after update roll pitch yaw (deg) = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

  cout << "\n------------ test_result -------------" << endl;
  // 如果完成正确,test_result应为0 0 1
  Eigen::Vector3d test_result = orientation_ * gravity_vector_;
  cout << "test_result x y z = " << test_result.transpose() << endl;
  cout << "test_result normalized x y z = " << test_result.normalized().transpose() << endl;

  cout << "\nz(): " << (orientation_ * gravity_vector_).z() << endl;
  cout << "norm z(): " << (orientation_ * gravity_vector_).normalized().z() << endl;
}

double DegToRad(double deg) { return M_PI * deg / 180.; }

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // note: 地固坐标系(Earth-Fixed Coordinate System)也称地球坐标系，
  // 是固定在地球上与地球一起旋转的坐标系。
  // 如果忽略地球潮汐和板块运动，地面上点的坐标值在地固坐标系中是固定不变的。

  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

void ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude) {
  // const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Vector3d translation(0,5,0);
  cout << "translation x y z = " << translation.transpose() << endl;

  Eigen::Quaterniond rotation( 0.948, 0 ,0, 0.317);
  // const Eigen::Quaterniond rotation =
  //     Eigen::AngleAxisd(DegToRad(latitude - 90.),
  //                       Eigen::Vector3d::UnitY()) *
  //     Eigen::AngleAxisd(DegToRad(-longitude),
  //                       Eigen::Vector3d::UnitZ());

  cout << "rotation: "
       << rotation.w() << " "
       << rotation.x() << " "
       << rotation.y() << " "
       << rotation.z() << endl;
  Eigen::Vector3d euler_angles_delta;
  euler_angles_delta = rotation.matrix().eulerAngles(0, 1, 2);
  cout << "\nrotation matrix to roll pitch yaw (deg) = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

      Eigen::Vector3d local = rotation * -translation;

  cout << "local x y z = " << local.transpose() << endl;

  // return cartographer::transform::Rigid3d(rotation * -translation, rotation);
  cout << " " << endl;
}

void test_fix_data()
{
  ComputeLocalFrameFromLatLong(39, 116);
}


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

void test_rigid()
{
  Rigid2 lhs(Eigen::Matrix<double, 2, 1>(3, 0), Eigen::Rotation2D<double>(0) );
  Rigid2 rhs(Eigen::Matrix<double, 2, 1>(1, 0), Eigen::Rotation2D<double>(0.0) );
  // lhs * rhs;
  // lhs.inverse();
  
  Rigid2 sensor_to_tracking(Eigen::Matrix<double, 2, 1>(1, 0), Eigen::Rotation2D<double>(0) );
  sensor_to_tracking.inverse();
  lhs * sensor_to_tracking.inverse();
}

int main()
{
  // testodom();
  // test();
  // test2();
  // test3();
  // test_lambda();
  // test_lower_bound();
  // test_Quaterniond();
  // test_imu_tracker();
  // test_fix_data();
  test_rigid();

  return 0;
}