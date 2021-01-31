#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

#include <tf/transform_datatypes.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_tf"); //Init ROS

    tf2::Quaternion q;
    q.setX(0);
    q.setW(1);
    q.setRotation(tf2::Vector3(0, 0, 1), M_PI / 2);
    q.setRPY(0, 0, M_PI / 2);
    tf2::Quaternion norm_q = q.normalize();
    // std::cout << "x: " << norm_q.x() << " y: " << norm_q.y()
    //     << " z: " << norm_q.z() << " w: " << norm_q.w() << std::endl;

    tf2::Transform baselink_to_laserlink(
        tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI),
        tf2::Vector3(0.5, 0, 0));

    tf2::Transform coord1_in_laserlink(
        tf2::Quaternion(tf2::Vector3(0, 0, 1), 0),
        tf2::Vector3(1, 1, 0));

    tf2::Transform coord1_in_baselink = baselink_to_laserlink * coord1_in_laserlink * baselink_to_laserlink.inverse();

    std::cout << "base in odom x: " << coord1_in_baselink.getOrigin().x() 
              << " , y: " << coord1_in_baselink.getOrigin().y()
              << " , z: " << coord1_in_baselink.getOrigin().z() << std::endl;
    std::cout << "theta: " << tf2::getYaw(coord1_in_baselink.getRotation()) * 180 / M_PI << std::endl;


    tf::Transform base_to_laser_(
        tf::Quaternion(tf::Vector3(1, 0, 0), M_PI ),
        tf::Vector3(0, 0, 1));

    tf::Transform coord2_in_laserlink(
        tf::Quaternion(tf::Vector3(0, 0, 1), 0 ),
        tf::Vector3(1, 2, 3));

    // tf::Transform coord2_in_baselink = base_to_laser_ * coord2_in_laserlink * base_to_laser_.inverse();
    tf::Transform coord2_in_baselink = base_to_laser_ * coord2_in_laserlink;
    tf::Vector3 pointPosBaseFrame(base_to_laser_ * tf::Vector3(
        coord2_in_laserlink.getOrigin().x(), coord2_in_laserlink.getOrigin().y(), coord2_in_laserlink.getOrigin().y()));

    std::cout << "coord2_in_baselink x: " << coord2_in_baselink.getOrigin().x() 
              << " , y: " << coord2_in_baselink.getOrigin().y()
              << " , z: " << coord2_in_baselink.getOrigin().z() << std::endl;
    std::cout << "theta: " << tf::getYaw(coord2_in_baselink.getRotation()) * 180 / M_PI << std::endl;


    return 0;
};