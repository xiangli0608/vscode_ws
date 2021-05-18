#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char WINDOW[] = "Image window";
static void help()
{
    printf("\nThis program demonstrates converting OpenCV Image to ROS Image messages  \n");
}

int main(int argc, char **argv)
{
    help();
    ros::init(argc, argv, "image_converter");

    //Reading an image from the file
    cv::Mat cv_image = cv::imread("/home/lx/0-ws/vscode_ws/src/test_opencv/src/blank_map.pgm");
    if (cv_image.empty())
    {
        ROS_ERROR("Read the picture failed!");
        return -1;
    }

    // cv::Rect r(250, 250, 120, 200);
    // cv::rectangle(cv_image, r, cv::Scalar(0, 0, 0), 3);

    using namespace cv;
    int lineType = 8;
    int w = 600;
    /** 创建一些点 */
    Point rook_points[1][20];
    rook_points[0][0] = Point(w / 4.0, 7 * w / 8.0);
    rook_points[0][1] = Point(3 * w / 4.0, 7 * w / 8.0);
    rook_points[0][2] = Point(3 * w / 4.0, 13 * w / 16.0);
    rook_points[0][3] = Point(11 * w / 16.0, 13 * w / 16.0);
      rook_points[0][4] = Point( 19*w/32.0, 3*w/8.0 );
      rook_points[0][5] = Point( 3*w/4.0, 3*w/8.0 );
      rook_points[0][6] = Point( 3*w/4.0, w/8.0 );
      rook_points[0][7] = Point( 26*w/40.0, w/8.0 );
      rook_points[0][8] = Point( 26*w/40.0, w/4.0 );
      rook_points[0][9] = Point( 22*w/40.0, w/4.0 );
      rook_points[0][10] = Point( 22*w/40.0, w/8.0 );
      rook_points[0][11] = Point( 18*w/40.0, w/8.0 );
      rook_points[0][12] = Point( 18*w/40.0, w/4.0 );
      rook_points[0][13] = Point( 14*w/40.0, w/4.0 );
      rook_points[0][14] = Point( 14*w/40.0, w/8.0 );
      rook_points[0][15] = Point( w / 4.0, w/8.0 );
      rook_points[0][16] = Point( w / 4.0, 3*w/8.0 );
      rook_points[0][17] = Point( 13*w/32.0, 3*w/8.0 );
      rook_points[0][18] = Point( 5*w/16.0, 13*w/16.0 );
    //   rook_points[0][19] = Point( w / 4.0, 13*w/16.0) ;

    const Point *ppt[1] = {rook_points[0]};
    int npt[] = {19};

    fillPoly(cv_image,
             ppt,
             npt,
             1,
             Scalar(0, 0, 0),
             lineType);

    //Show the image
    cv::namedWindow(WINDOW);
    cv::imshow(WINDOW, cv_image);
    cv::waitKey(0);

    //   ros::spin();
    return 0;
}