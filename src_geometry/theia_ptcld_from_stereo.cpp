// pointcloid from stereo pair.
#include <iostream>
#include <string>
#include <vector>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <theia/theia.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

#include "../utils/MiscUtils.h"
#include "../utils/ElapsedTime.h"
#include "../utils/RawFileIO.h"
#include "../utils/PoseManipUtils.h"
#include "../utils/RosMarkerUtils.h"
#include "../utils/TermColor.h"

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"

#include "gms_matcher.h"

// const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_1loop_in_lab/";
const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_multi_loops_in_lab/";


int main(int argc, char ** argv)
{
    //
    // ROS init
    ros::init(argc, argv, "theia_ptcld_from_stereo" );
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("chatter", 1000);
    ros::Rate loop_rate(10);

    //
    // Load Camera Parameters
    camodocal::CameraPtr m_camera_left, m_camera_right;
    // m_camera_left = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    // m_camera_right = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");

    m_camera_left = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string("/app/catkin_ws/src/vins_mono_debug_pkg/build/stereo")+"/camera_left.yaml");
    m_camera_right = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string("/app/catkin_ws/src/vins_mono_debug_pkg/build/stereo")+"/camera_right.yaml");

    

    assert( m_camera_right && m_camera_left );
    cout << m_camera_left->parametersToString() << endl;
    cout << m_camera_right->parametersToString() << endl;

    ElapsedTime timer;
    int frame_id = 264;
    while( ros::ok() )
    {

    //
    // Load Left Image
    cout << "READ IMAGE : " << BASE+"/"+std::to_string(frame_id)+".jpg" << endl;
    cv::Mat im_left = cv::imread( BASE+"/"+std::to_string(frame_id)+".jpg" );
    if( im_left.rows == 0 || im_left.cols == 0 )
        continue;
    cv::imshow( "left", im_left );


    //
    // Load Right Image
    cout << "READ IMAGE : " << BASE+"/"+std::to_string(frame_id)+"_1.jpg" << endl;
    cv::Mat im_right = cv::imread( BASE+"/"+ std::to_string(frame_id)+"_1.jpg" );
    if( im_right.rows == 0 || im_right.cols == 0 )
        continue;
    cv::imshow( "right", im_right );

    Matrix4d wTc;
    if( !RawFileIO::read_eigen_matrix( BASE+"/"+to_string(frame_id)+".wTc" , wTc ) )
        continue;


    //
    // Point feature and descriptors extract
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat d1, d2;

	cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
	orb->setFastThreshold(0);

    timer.tic();
	orb->detectAndCompute(im_left, Mat(), kp1, d1);
	orb->detectAndCompute(im_right, Mat(), kp2, d2);
    cout << "2X detectAndCompute(ms) : " << timer.toc_milli() << endl;
    std::cout << "d1 " << MiscUtils::cvmat_info( d1 ) << std::endl;
    std::cout << "d2 " << MiscUtils::cvmat_info( d2 ) << std::endl;

    //plot
    cv::Mat dst_left, dst_right;
    MatrixXd e_kp1, e_kp2;
    MiscUtils::keypoint_2_eigen( kp1, e_kp1 );
    MiscUtils::keypoint_2_eigen( kp2, e_kp2 );
    MiscUtils::plot_point_sets( im_left, e_kp1, dst_left, cv::Scalar(0,0,255), false );
    MiscUtils::plot_point_sets( im_right, e_kp2, dst_right, cv::Scalar(0,0,255), false );
    cv::imshow( "dst_left", dst_left );
    cv::imshow( "dst_right", dst_right );

    //
    // Point feature matching
    cv::BFMatcher matcher(NORM_HAMMING); // TODO try FLANN matcher here.
    vector<DMatch> matches_all, matches_gms;
    timer.tic();
	matcher.match(d1, d2, matches_all);
    std::cout << "BFMatcher : npts = " << matches_all.size() << std::endl;
    std::cout << "BFMatcher took (ms) : "<< timer.toc_milli() << std::endl;

    // gms_matcher
    timer.tic();
    std::vector<bool> vbInliers;
	gms_matcher gms(kp1, im_left.size(), kp2, im_right.size(), matches_all);
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	cout << "Get total " << num_inliers << " matches." << endl;
    cout << "GMSMatcher took (ms) " << timer.toc_milli() << std::endl;

	// collect matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		if (vbInliers[i] == true)
		{
			matches_gms.push_back(matches_all[i]);
		}
	}
    MatrixXd M1, M2;
    MiscUtils::dmatch_2_eigen( kp1, kp2, matches_gms, M1, M2, false );
    cv::Mat dst;
    MiscUtils::plot_point_pair( im_left, M1, -1, im_right, M2, -1, dst, 0 );
    cv::imshow( "dst", dst );

    cv::waitKey(0);

    //
    // Triangulate
    //  a) image cords --> liftProjective (undistort the imaged point)
    //  b) pose1, pose2 --> stereo baseline
    //  c) theia-sfm.triangulate()
    //  d) visualize 3d points on rviz

    // ------b
    Matrix4d pose1; // w_T_{left}
    pose1 = Matrix4d::Identity();
    // pose1 = wTc;
    Matrix4d pose2;     // find best estimate for initial transform from left camera frame to right camera frame ==> words from `camdocal/calib/StereoCameraCalibration.cc`
    // so pose2 is {right}_T_{left}
        //transform:
        // q_x: -1.8252509868889259e-04
        // q_y: -1.6291774489779708e-03
        // q_z: -1.2462127842978489e-03
        // q_w: 9.9999787970731446e-01
        // t_x: -1.2075905420832895e+02
        // t_y: 5.4110610639412482e-01
        // t_z: 2.4484815673909591e-01
    double _rot_xyzw[] = {-1.8252509868889259e-04,-1.6291774489779708e-03,-1.2462127842978489e-03,9.9999787970731446e-01};
    double _tr_xyz[] = {-1.2075905420832895e+02/1000.,5.4110610639412482e-01/1000.,2.4484815673909591e-01/1000.};
    // double _tr_xyz[] = {-1.2075905420832895e+02,5.4110610639412482e-01,2.4484815673909591e-01};

    PoseManipUtils::raw_xyzw_to_eigenmat( _rot_xyzw, _tr_xyz, pose2 );
    Matrix4d pose2_inv = pose1 * pose2.inverse(); // pose2_inv : {left}_T_{right}
    cout << "pose2\n" << pose2_inv << endl;

    Matrix<double,3,4> _pose1_3x4, _pose2_3x4;
    _pose1_3x4 = pose1.topRows(3);
    _pose2_3x4 = pose2.topRows(3);
    // _pose2_3x4 = pose2_inv.topRows(3); // if u do this reprojection is wrong.
    cout << "_pose1_3x4\n" << _pose1_3x4 << endl;
    cout << "_pose2_3x4\n" << _pose2_3x4 << endl;




    MatrixXd triangulated_pts = MatrixXd::Zero( 3, M1.cols() );
    MatrixXd triangulated_pts_colors = MatrixXd::Zero( 3, M1.cols() );
    timer.tic();
    for( int k=0 ; k<M1.cols() ; k++ ) { // for each imaged point
        cout << TermColor::RED() << "---" << k << "---" << TermColor::RESET() << "\n";
        // -----a
        Vector2d im1_cord = M1.col(k);
        cout << "im1_cord:" << im1_cord.transpose() << endl;
        Vector3d sphere1_cord;
        m_camera_left->liftProjective( im1_cord, sphere1_cord );
        sphere1_cord = sphere1_cord / sphere1_cord(2);
        cout << "sphere1_cord:" << sphere1_cord.transpose()  << endl;


        Vector2d im2_cord = M2.col(k);
        cout << "im2_cord:" << im2_cord.transpose() << endl;
        Vector3d sphere2_cord;
        m_camera_right->liftProjective( im2_cord, sphere2_cord );
        sphere2_cord = sphere2_cord / sphere2_cord(2);
        cout << "sphere2_cord:" << sphere2_cord.transpose()  << endl;

        Vector2d _im1_nrm, _im2_nrm;
        _im1_nrm = sphere1_cord.topRows(2);
        _im2_nrm = sphere2_cord.topRows(2);



        // -----c
        Vector4d _3dpt;
        theia::Triangulate( _pose1_3x4, _pose2_3x4, _im1_nrm, _im2_nrm, &_3dpt );
        triangulated_pts.col(k) = _3dpt.topRows(3) / _3dpt(3);
        cout << "_3dpt:" << _3dpt.transpose()<< "\t";
        cout << "_3dpt_nonhomo:" << triangulated_pts.col(k).transpose() << endl;


        // cv::Vec3b _col = im_left.at<cv::Vec3d>( (int)im1_cord(1), (int)im1_cord(0) );
        uchar _col = im_left.at<uchar>( (int)im1_cord(1), (int)im1_cord(0) );
        triangulated_pts_colors(0,k) = double(_col)/ 255.;
        triangulated_pts_colors(1,k) = double(_col)/ 255.;
        triangulated_pts_colors(2,k) = double(_col)/ 255.;
        // cout << "_3dpt_color: " << triangulated_pts_colors.col(k).transpose() << endl;
    }

    cout << "Triangulation of " << M1.cols() << " pts done in (ms): " << timer.toc_milli() << endl;


    // ------d : visualization
    // -------------------- reproject 3d points
    MatrixXd reprojected_pts = MatrixXd::Zero( 2, M1.cols() );
    for( int k=0 ; k<M1.cols() ; k++ ) {
        Vector3d P = triangulated_pts.col(k);
        Vector2d p;
        // m_camera_left->spaceToPlane( P, p );
        m_camera_right->spaceToPlane( P, p );
        reprojected_pts.col(k) = p;
        cout << k << " P:" << P.transpose() << "\t";
        cout << "p:" << p.transpose() << endl;
    }
    // MiscUtils::plot_point_sets( dst_left, reprojected_pts, cv::Scalar(255,0,0), false, "reprohected in blue" );
    // cv::imshow( "dst_left", dst_left );
    MiscUtils::plot_point_sets( dst_right, reprojected_pts, cv::Scalar(255,0,0), false, "reprohected in blue" );
    cv::imshow( "dst_right", dst_right );
    cv::waitKey(0);



    // -------------------- cam pose
    visualization_msgs::Marker cam_left, cam_right;
    RosMarkerUtils::init_camera_marker( cam_left, -1.0 );
    cam_left.ns = "camera";
    cam_left.id = 2*frame_id + 0;
    RosMarkerUtils::setpose_to_marker( pose1, cam_left );
    RosMarkerUtils::init_camera_marker( cam_right, -1.0 );
    cam_right.ns = "camera";
    cam_right.id = 2*frame_id + 1;
    RosMarkerUtils::setpose_to_marker( pose2_inv, cam_right );
    RosMarkerUtils::setcolor_to_marker( 1.0, 1.0, 1.0, cam_left );
    RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.0, cam_right );

    // --------------------- ptcld
    visualization_msgs::Marker ptcld_marker;
    RosMarkerUtils::init_points_marker( ptcld_marker );
    ptcld_marker.ns = "wX";
    ptcld_marker.id = frame_id + 0;
    ptcld_marker.scale.x = 0.02;
    ptcld_marker.scale.y = 0.02;
    RosMarkerUtils::add_points_to_marker( triangulated_pts, ptcld_marker );
    RosMarkerUtils::add_colors_to_marker( triangulated_pts_colors, ptcld_marker );

    chatter_pub.publish( cam_left );
    chatter_pub.publish( cam_right );
    chatter_pub.publish( ptcld_marker );
    ros::spinOnce();
    loop_rate.sleep();
    frame_id++;
    break;

    }





}
