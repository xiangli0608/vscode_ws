#include <stdio.h>
#include <cmath>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/LU>



int main()
{
    float mapResolution = 0.05;
    float scaleToMap = 1 / mapResolution;

    float totalMapSizeX = mapResolution * static_cast<float>(2048); // 实际物理尺寸范围
    float mid_offset_x = totalMapSizeX * 0.5;               ///  offset 的计算存在问题吧 ????


    Eigen::Translation2f topleft(51.2, 51.2);
    Eigen::AlignedScaling2f scale(scaleToMap, scaleToMap);
    Eigen::Affine2f mapTworld = scale * topleft;
    std::cout << "mapTworld\n" << mapTworld.matrix() << std::endl;

    Eigen::Vector2f world(1, 1);
    Eigen::Vector2f map = mapTworld * world;
    return 0;
}