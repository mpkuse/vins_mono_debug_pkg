/** Experiments to project 3d points on images.
*/

#include <iostream>

#include "utils/RawFileIO.h"
#include "utils/PoseManipUtils.h"
#include "utils/MiscUtils.h"

#include "PinholeCamera.h"

const string base_path = "/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/";

bool load_i( int i, cv::Mat& im, Matrix4d& wTc, MatrixXd& uv, MatrixXd& cX )
{
    string fname = base_path+"/"+std::to_string( i );

    // Image
    im = cv::imread( fname+".jpg" );
    // cv::imshow( "win", im );

    // VINS-pose (odometry)
    RawFileIO::read_eigen_matrix( fname+".wTc", wTc );

    // 3D points - in camera cords
    RawFileIO::read_eigen_matrix( fname+".cX.pointcloud", cX );

    // 2D points - tracked points
    RawFileIO::read_eigen_matrix( fname+".uv", uv );

}

bool load_i( int i, cv::Mat& im, Matrix4d& wTc, MatrixXd& uv, MatrixXd& cX, VectorXi& uv_ids )
{
    string fname = base_path+"/"+std::to_string( i );

    // Image
    im = cv::imread( fname+".jpg" );
    // cv::imshow( "win", im );

    // VINS-pose (odometry)
    RawFileIO::read_eigen_matrix( fname+".wTc", wTc );

    // 3D points - in camera cords
    RawFileIO::read_eigen_matrix( fname+".cX.pointcloud", cX );

    // 2D points - tracked points
    RawFileIO::read_eigen_matrix( fname+".uv", uv );

    // IDs
    RawFileIO::read_eigen_matrix( fname+".id", uv_ids );

}

int main()
{
    //
    // Load Camera - Blackbbox4 camera (need to change this accordingly for other cameras)
    PinholeCamera camera( 274.127, 272.912,327.605 , 253.85,
                            512, 640,
                            -0.233481, 0.037218, -0.000236, -0.001222
                        );

    // Load 1st
    cv::Mat im_1;
    Matrix4d wTc_1;
    MatrixXd uv_1, cX_1;
    VectorXi uv_id_1;
    load_i( 533, im_1, wTc_1, uv_1, cX_1, uv_id_1 );
    // cv::imshow( "533", im_1 );
    {
        cv::Mat dst;
        MiscUtils::plot_point_sets( im_1, uv_1, dst, cv::Scalar(255,0,0), uv_id_1, "self uv" );
        cv::imshow( "533", dst );
    }


    // Load 2nd
    cv::Mat im_2;
    Matrix4d wTc_2;
    MatrixXd uv_2, cX_2;
    VectorXi uv_id_2;
    // load_i( 536, im_2, wTc_2, uv_2, cX_2, uv_id_2 );
    // load_i( 887, im_2, wTc_2, uv_2, cX_2 );
    load_i( 1637, im_2, wTc_2, uv_2, cX_2, uv_id_2 );
    // cv::imshow( "535", im_2 );
    {
        cv::Mat dst;
        MiscUtils::plot_point_sets( im_2, uv_2, dst, cv::Scalar(255,0,0), true, "self uv" );
        MiscUtils::plot_point_sets( dst, uv_2, cv::Scalar(255,255,255), uv_id_2, "self uv" );
        cv::imshow( "1637", dst );
    }


    // plot these 2d points
    cv::Mat dst;
    // im_1.copyTo(dst);
    MiscUtils::plot_point_sets( im_1, uv_1, dst, cv::Scalar(0,0,255), true, "in red uv" );


    MatrixXd PI_cX_1;
    camera.perspectiveProject3DPoints( cX_1, PI_cX_1 );
    MiscUtils::plot_point_sets( dst, PI_cX_1, cv::Scalar(0,255,255), true, ";in yellow reproj" );


    MatrixXd __a;
    MatrixXd c1_X2 = wTc_1.inverse() * wTc_2 * cX_2;
    camera.perspectiveProject3DPoints( c1_X2, __a );
    MiscUtils::plot_point_sets( dst, __a, cv::Scalar(0,255,0), true, ";;in green reproj of 2nd 3d pts on 1st cam" );


    cv::imshow( "dst", dst );



    cv::waitKey(0);
    return 0;

}
