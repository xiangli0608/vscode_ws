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
    Eigen::Vector3d start(0.0 , 1.0 , 2.0);
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
    const int c = [&a, b]()->int{
        std::cout << "a: " << a << " b: " << b << std::endl;
        a = 3;
        return a + b;
    }();

    std::cout << "c: " << c << std::endl;

    std::cout << std::endl;

}

void test_lower_bound()
{
  	double a[]={1,2,3,4,5,7,8,9};
    auto it = lower_bound(a, a+8, 2.3, [](const double a, const double b) {
        return a < b;
      });
    cout << "lower_bound: " << *it << endl;
}

void test_Quaterniond()
{
  double w =  0.991, x = 0.0, y = 0.0, z =  0.131;
  Eigen::Quaterniond last_orientation(w, x, y, z);
  Eigen::Vector3d euler_angles_last = last_orientation.toRotationMatrix().eulerAngles(0,1,2);
  cout << "last pitch yaw roll = " << euler_angles_last.transpose() * 180 / M_PI << endl;

  w =  0.974, x = 0.0, y = 0.0, z = 0.225;
  Eigen::Quaterniond new_orientation(w, x, y, z);
  Eigen::Vector3d euler_angles_new = new_orientation.toRotationMatrix().eulerAngles(0,1,2);
  cout << "new pitch yaw roll = " << euler_angles_new.transpose() * 180 / M_PI << endl;

  Eigen::Quaterniond rotate_delta = last_orientation.inverse() * new_orientation;
  Eigen::Vector3d euler_angles_delta = rotate_delta.toRotationMatrix().eulerAngles(0,1,2);
  cout << "delta pitch yaw roll = " << euler_angles_delta.transpose() * 180 / M_PI << endl;

  new_orientation = last_orientation * rotate_delta;
  euler_angles_new = new_orientation.toRotationMatrix().eulerAngles(0,1,2);
  cout << "new 2 pitch yaw roll = " << euler_angles_new.transpose() * 180 / M_PI << endl;

}



int main()
{
    // testodom();
    // test();
    // test2();
    // test3();
    // test_lambda();
    // test_lower_bound();
    test_Quaterniond();



    return 0;
}