// Uses my CameraGeometry classes to compute disparity map, 3d points and view
// those points in rviz and verify. Also attempty to compute relative pose in this
// given a loop candidate

#include <iostream>
#include <string>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../utils/CameraGeometry.h"


#include "../utils/MiscUtils.h"
#include "../utils/ElapsedTime.h"
#include "../utils/PoseManipUtils.h"
#include "../utils/TermColor.h"
#include "../utils/RawFileIO.h"
#include "../utils/RosMarkerUtils.h"

#include "gms_matcher.h"
#include <theia/theia.h>



void point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud )
{
    ElapsedTime timer;


    //
    // Point feature and descriptors extract
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat d1, d2; //< descriptors

    cv::Ptr<cv::ORB> orb = cv::ORB::create(10000);
    orb->setFastThreshold(0);

    timer.tic();
    orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
    orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);
    cout << "2X detectAndCompute(ms) : " << timer.toc_milli() << endl;
    // std::cout << "d1 " << MiscUtils::cvmat_info( d1 ) << std::endl;
    // std::cout << "d2 " << MiscUtils::cvmat_info( d2 ) << std::endl;

    //plot
    // cv::Mat dst_left, dst_right;
    // MatrixXd e_kp1, e_kp2;
    // MiscUtils::keypoint_2_eigen( kp1, e_kp1 );
    // MiscUtils::keypoint_2_eigen( kp2, e_kp2 );
    // MiscUtils::plot_point_sets( imleft_undistorted, e_kp1, dst_left, cv::Scalar(0,0,255), false );
    // MiscUtils::plot_point_sets( imright_undistorted, e_kp2, dst_right, cv::Scalar(0,0,255), false );
    // cv::imshow( "dst_left", dst_left );
    // cv::imshow( "dst_right", dst_right );

    //
    // Point feature matching
    cv::BFMatcher matcher(cv::NORM_HAMMING); // TODO try FLANN matcher here.
    vector<cv::DMatch> matches_all, matches_gms;
    timer.tic();
    matcher.match(d1, d2, matches_all);
    std::cout << "BFMatcher : npts = " << matches_all.size() << std::endl;
    std::cout << "BFMatcher took (ms) : "<< timer.toc_milli() << std::endl;


    // gms_matcher
    timer.tic();
    std::vector<bool> vbInliers;
    gms_matcher gms(kp1, imleft_undistorted.size(), kp2, imright_undistorted.size(), matches_all);
    int num_inliers = gms.GetInlierMask(vbInliers, false, false);
    cout << "Got total gms matches " << num_inliers << " matches." << endl;
    cout << "GMSMatcher took (ms) " << timer.toc_milli() << std::endl;

    // collect matches
    for (size_t i = 0; i < vbInliers.size(); ++i)
    {
        if (vbInliers[i] == true)
        {
            matches_gms.push_back(matches_all[i]);
        }
    }
    // MatrixXd M1, M2;
    MiscUtils::dmatch_2_eigen( kp1, kp2, matches_gms, u, ud, true );


}



int stereo_demo_easy()
{
    ElapsedTime timer;
    const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_multi_loops_in_lab/";
    // const std::string BASE = "/Bulk_Data/ros_bags/bluefox_stereo/calib/leveled_cam_sampled/";

    //--------- Intrinsics load
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");
    // camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_left.yaml");
    // camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_right.yaml");

    cout << left_camera->parametersToString() << endl;
    cout << right_camera->parametersToString() << endl;


    //----------- Stereo Base line load (alsoed called extrinsic calibration)
        // mynt eye
    Vector4d q_wxyz = Vector4d( -1.8252509868889259e-04,-1.6291774489779708e-03,-1.2462127842978489e-03,9.9999787970731446e-01 );
    Vector3d tr_xyz = Vector3d( -1.2075905420832895e+02/1000.,5.4110610639412482e-01/1000.,2.4484815673909591e-01/1000. );

        // bluefox stereo leveled
    // Vector4d q_wxyz = Vector4d( -1.7809713490350254e-03, 4.2143149583451564e-04,4.1936662160154632e-02, 9.9911859501433165e-01 );
    // Vector3d tr_xyz = Vector3d( -1.4031938291177164e+02/1000.,-6.6214729932530441e+00/1000.,1.4808567571722902e+00/1000. );

    Matrix4d right_T_left;
    PoseManipUtils::raw_xyzw_to_eigenmat( q_wxyz, tr_xyz, right_T_left );
    // cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;



    //-------------------- init stereogeom
    std::shared_ptr<StereoGeometry> stereogeom;
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );
    stereogeom->set_K( 375.0, 375.0, 376.0, 240.0 );



    int frame_id = 1005;
    //----------------- load images_raw for left and right
    // for( frame_id=0; frame_id < 2500 ;  frame_id++ )
    //{
    cout << "READ IMAGE " << frame_id << endl;
    cv::Mat imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+".jpg", 0 );
    cv::Mat imright_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+"_1.jpg", 0 );
    // cv::Mat imleft_raw =  cv::imread( BASE+"/cam0_"+std::to_string(frame_id)+".png",0 );
    // cv::Mat imright_raw = cv::imread( BASE+"/cam1_"+ std::to_string(frame_id)+".png",0 );

    // if( imleft_raw.empty() || imright_raw.empty() )
        // continue;

    //------------------- stereogeom->get3dpoints_from_raw_images()
    //      can use one of the options depending on the need.
    #if 0
    // (A) fastest - if you are just looking for valid 3d points - look at the CameraGeometry.h header to see various options to call.
    timer.tic();
    MatrixXd _3dpts; //4xN
    stereogeom->get3dpoints_from_raw_images(imleft_raw, imright_raw, _3dpts );
    cout << timer.toc_milli() << " (ms)!!  get3dpoints_from_raw_images\n";

    cout << "_3dpts.shape= " << _3dpts.rows() << " " << _3dpts.cols() << endl;
    #endif


    #if 0
    // will get the 3d points and disparity. Takes about 2-4ms more than (A)
    MatrixXd _3dpts; //4xN
    cv::Mat disparity_for_visualization;
    cout << "_3dpts.shape= " << _3dpts.rows() << " " << _3dpts.cols() << endl;
    timer.tic();
    stereogeom->get3dpoints_and_disparity_from_raw_images(imleft_raw, imright_raw, _3dpts, disparity_for_visualization );
    cout << timer.toc_milli() << " (ms)!!  get3dpoints_and_disparity_from_raw_images\n";

    cv::imshow( "disparity_for_visualization", disparity_for_visualization );
    #endif


    #if 1
    // will get 3d points, stereo-rectified image, and disparity false colormap
    MatrixXd _3dpts; //4xN
    cv::Mat imleft_srectified, imright_srectified;
    cv::Mat disparity_for_visualization;

    timer.tic();
    stereogeom->get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(imleft_raw, imright_raw,
        imleft_srectified, imright_srectified,
         _3dpts, disparity_for_visualization );
    cout << timer.toc_milli() << " (ms)!!  get_srectifiedim_and_3dpoints_and_disparity_from_raw_images\n";

    cv::imshow( "imleft_srectified", imleft_srectified );
    cv::imshow( "imright_srectified", imright_srectified );
    cv::imshow( "disparity_for_visualization", disparity_for_visualization );
    #endif


    //-------------------- reproject the 3d points.
    //      note: that these 3d points after reprojections will be correct as plotted to imleft_srectified
    MatrixXd reproj_uv;
    GeometryUtils::idealProjection( stereogeom->get_K(), _3dpts, reproj_uv );

    cv::Mat dst_reproj_uv;
    vector<cv::Scalar> pt_colors_cv_scalar;
    GeometryUtils::depthColors( _3dpts, pt_colors_cv_scalar, .5, 4.5 );
    MiscUtils::plot_point_sets( imleft_srectified, reproj_uv, dst_reproj_uv, pt_colors_cv_scalar, 0.6, "plot of reprojected points;colored by depth" );
    cv::imshow( "dst_reproj_uv" , dst_reproj_uv );

    //-------------------- visualize 3d points with rviz

    cv::waitKey(0);
    //}


    // TODO run this inside the loop for frame_id
    // ====== ROS init
    int argc=0;
    char ** argv;
    ros::init(argc, argv, "camera_geometry_inspect_ptcld" );
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("chatter", 1000);
    ros::Rate loop_rate(10);


    // ======= cam pose
    visualization_msgs::Marker cam_left, cam_right;
    RosMarkerUtils::init_camera_marker( cam_left, 1.0 );
    cam_left.ns = "camera";
    cam_left.id = 2*frame_id + 0;
    RosMarkerUtils::setpose_to_marker( Matrix4d::Identity(), cam_left );
    RosMarkerUtils::init_camera_marker( cam_right, 1.0 );
    cam_right.ns = "camera";
    cam_right.id = 2*frame_id + 1;
    RosMarkerUtils::setpose_to_marker( right_T_left.inverse(), cam_right );
    RosMarkerUtils::setcolor_to_marker( 1.0, 1.0, 1.0, cam_left );
    RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.0, cam_right );

    // =========== ptcld
    visualization_msgs::Marker ptcld_marker;
    RosMarkerUtils::init_points_marker( ptcld_marker );
    ptcld_marker.ns = "wX";
    ptcld_marker.id = frame_id + 0;
    ptcld_marker.scale.x = 0.02;
    ptcld_marker.scale.y = 0.02;

    RosMarkerUtils::add_points_to_marker( _3dpts, ptcld_marker );

    MatrixXd pt_colors;
    GeometryUtils::depthColors( _3dpts, pt_colors, .5, 4.5 );
    RosMarkerUtils::add_colors_to_marker( pt_colors, ptcld_marker );


    while( ros::ok() )
    {
        chatter_pub.publish( cam_left );
        chatter_pub.publish( cam_right );
        chatter_pub.publish( ptcld_marker );
        ros::spinOnce();
        loop_rate.sleep();
    }


}

const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_pinhole_1loop_in_lab/";


void relative_pose_compute_with_theia( int frame_a, int frame_b, Matrix4d& out_b_T_a )
{
    ElapsedTime timer;
    // const std::string BASE = "/Bulk_Data/ros_bags/bluefox_stereo/calib/mynt_pinhole_1loop_in_lab/";

    //--------- Intrinsics load
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");
    // camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_left.yaml");
    // camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_right.yaml");

    cout << left_camera->parametersToString() << endl;
    cout << right_camera->parametersToString() << endl;


    //----------- Stereo Base line load (alsoed called extrinsic calibration)
        // mynt eye
    Vector4d q_wxyz = Vector4d( -1.8252509868889259e-04,-1.6291774489779708e-03,-1.2462127842978489e-03,9.9999787970731446e-01 );
    Vector3d tr_xyz = Vector3d( -1.2075905420832895e+02/1000.,5.4110610639412482e-01/1000.,2.4484815673909591e-01/1000. );

        // bluefox stereo leveled
    // Vector4d q_wxyz = Vector4d( -1.7809713490350254e-03, 4.2143149583451564e-04,4.1936662160154632e-02, 9.9911859501433165e-01 );
    // Vector3d tr_xyz = Vector3d( -1.4031938291177164e+02/1000.,-6.6214729932530441e+00/1000.,1.4808567571722902e+00/1000. );

    Matrix4d right_T_left;
    PoseManipUtils::raw_xyzw_to_eigenmat( q_wxyz, tr_xyz, right_T_left );
    // cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;



    //-------------------- init stereogeom
    std::shared_ptr<StereoGeometry> stereogeom;
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );
    stereogeom->set_K( 375.0, 375.0, 376.0, 240.0 );


    //------ 3d points from frame_a
    cv::Mat a_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_a)+".jpg", 0 );
    cv::Mat a_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_a)+"_1.jpg", 0 );
    cv::Mat a_imleft_srectified, a_imright_srectified;
    cv::Mat _3dImage;
    MatrixXd _3dpts;
    stereogeom->get3dpoints_and_3dmap_from_raw_images( a_imleft_raw, a_imright_raw,
                        _3dpts, _3dImage,
                        a_imleft_srectified, a_imright_srectified );

    cout << "a_imleft_srectified : " << MiscUtils::cvmat_info(a_imleft_srectified ) << endl;
    cout << "a_imright_srectified : " << MiscUtils::cvmat_info(a_imright_srectified ) << endl;
    cout << "_3dImage : " << MiscUtils::cvmat_info(_3dImage ) << endl;
    cout << "_3dpts : " << _3dpts.rows() << "x" << _3dpts.cols() << endl;

    // project _3dpts and vis on a_imright_srectified
    MatrixXd reproj_uv;
    GeometryUtils::idealProjection( stereogeom->get_K(), _3dpts, reproj_uv );
    cv::Mat dst_reproj_uv;
    vector<cv::Scalar> pt_colors_cv_scalar;
    GeometryUtils::depthColors( _3dpts, pt_colors_cv_scalar, .5, 4.5 );
    MiscUtils::plot_point_sets( a_imleft_srectified, reproj_uv, dst_reproj_uv,
            pt_colors_cv_scalar, 0.6, "plot of reprojected points on ima_left_srect;colored by depth" );
    cv::imshow( "dst_reproj_uv" , dst_reproj_uv );


    //------- feature matches between frame_a and frame_b
    cv::Mat b_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_b)+".jpg", 0 );
    cv::Mat b_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_b)+"_1.jpg", 0 );
    cv::Mat b_imleft_srectified, b_imright_srectified;
    stereogeom->do_stereo_rectification_of_raw_images( b_imleft_raw, b_imright_raw,
                            b_imleft_srectified, b_imright_srectified );

    // a_imleft_srectified <--> b_imleft_srectified
    MatrixXd u, ud; // u is from frame_a; ud is from frame_b
    point_feature_matches(a_imleft_srectified, b_imleft_srectified, u, ud );
    cv::imshow( "a_imleft_srectified", a_imleft_srectified );
    cv::imshow( "b_imleft_srectified", b_imleft_srectified );

    // cv::Mat dst_feat_matches;
    // MiscUtils::plot_point_pair( a_imleft_srectified, u, frame_a,
                    //  b_imleft_srectified, ud, frame_b,
                        // dst_feat_matches, 3, "gms_matcher" );
    // cv::imshow( "dst_feat_matches", dst_feat_matches);


    // pick say 100 feature matches which has valid depth
    int c = 0;
    MatrixXd ud_normalized = stereogeom->get_K().inverse() * ud;
    MatrixXd u_normalized = stereogeom->get_K().inverse() * u;
    std::vector<Eigen::Vector2d> feature_position;
    std::vector<Eigen::Vector3d> world_point;
    for( int k=0 ; k<u.cols() ; k++ )
    {
        cv::Vec3f _3dpt = _3dImage.at<cv::Vec3f>( (int)u(1,k), (int)u(0,k) );
        if( _3dpt[2] < 0.1 || _3dpt[2] > 25.  )
            continue;

        c++;
        cout << TermColor::RED() << "---" << k << "---" << TermColor::RESET() << endl;
        cout << "ud=" << ud.col(k).transpose() ;
        cout << " <--> ";
        cout << "u=" << u.col(k).transpose() ;
        cout << "  3dpt of u=";
        cout <<  TermColor::GREEN() << _3dpt[0] << " " << _3dpt[1] << " " << _3dpt[2] << " " << TermColor::RESET();
        cout << endl;

        feature_position.push_back( Vector2d( ud_normalized(0,k), ud_normalized(1,k) ) );
        // feature_position.push_back( Vector2d( u_normalized(0,k), u_normalized(1,k) ) );
        world_point.push_back( Vector3d( _3dpt[0], _3dpt[1], _3dpt[2] ) );
    }
    cout << "of the total " << u.cols() << " point feature correspondences " << c << " had valid depths\n";


    //------- 3d-2d pnp (theia::DlsPnp)
    std::vector<Eigen::Quaterniond> solution_rotations;
    std::vector<Eigen::Vector3d> solution_translations;
    theia::DlsPnp( feature_position, world_point, &solution_rotations, &solution_translations  );
    cout << "solutions count = " << solution_rotations.size() << " " << solution_translations.size() << endl;
    Matrix4d b_T_a = Matrix4d::Identity();
    b_T_a.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
    b_T_a.col(3).topRows(3) = solution_translations[0];
    cout << "solution_T " << b_T_a << endl;
    cout << "solution_T (b_T_a): " << PoseManipUtils::prettyprintMatrix4d( b_T_a ) << endl;



    //--- pose analysis


    GeometryUtils::idealProjection( stereogeom->get_K(), _3dpts, reproj_uv );
    GeometryUtils::depthColors( _3dpts, pt_colors_cv_scalar, .5, 4.5 );
    MiscUtils::plot_point_sets( b_imleft_srectified, reproj_uv, dst_reproj_uv,
            pt_colors_cv_scalar, 0.6, "PI( a_X );plot on imb;colored by depth" );
    cv::imshow( "dst_reproj_uv2" , dst_reproj_uv );


    GeometryUtils::idealProjection( stereogeom->get_K(), b_T_a * _3dpts, reproj_uv );
    GeometryUtils::depthColors( b_T_a * _3dpts, pt_colors_cv_scalar, .5, 4.5 );
    MiscUtils::plot_point_sets( b_imleft_srectified, reproj_uv, dst_reproj_uv,
            pt_colors_cv_scalar, 0.6, "PI( b_T_a * a_X );plot on imb;colored by depth" );
    cv::imshow( "dst_reproj_uv3" , dst_reproj_uv );
    // Perfect!

    cv::waitKey(0);

    out_b_T_a = b_T_a;




}



int main()
{
    // stereo_demo_easy();

    Matrix4d b_T_a;
    int frame_a = 840;
    int frame_b = 1344;
    relative_pose_compute_with_theia( frame_a, frame_b, b_T_a );


    ///////// ROS INIT
    int argc=0;
    char ** argv;
    ros::init(argc, argv, "camera_geometry_inspect_ptcld" );
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("chatter", 1000);
    ros::Rate loop_rate(10);


    //////////// Play with poses
    Matrix4d w_T_a, w_T_b;
    RawFileIO::read_eigen_matrix( BASE+"/"+to_string(frame_a)+".wTc", w_T_a );
    RawFileIO::read_eigen_matrix( BASE+"/"+to_string(frame_b)+".wTc", w_T_b );
    Matrix4d w_Tcap_b =  w_T_a * b_T_a.inverse();
    cout << "w_T_a: " << PoseManipUtils::prettyprintMatrix4d( w_T_a ) << endl;
    cout << "w_T_b: " << PoseManipUtils::prettyprintMatrix4d( w_T_b ) << endl;
    cout << "w_Tcap_b: " << PoseManipUtils::prettyprintMatrix4d( w_Tcap_b ) << endl;


    ///////////// Cam pose
    visualization_msgs::Marker cam_a, cam_b, cam_b_new;
    RosMarkerUtils::init_camera_marker( cam_a, 3.0 );
    cam_a.ns = "camera";
    cam_a.id = 0;
    RosMarkerUtils::setpose_to_marker( w_T_a, cam_a );
    RosMarkerUtils::setcolor_to_marker( 1.0, 0.0, 0.0, cam_a );

    RosMarkerUtils::init_camera_marker( cam_b, 3.0 );
    cam_b.ns = "camera";
    cam_b.id = 1;
    RosMarkerUtils::setpose_to_marker( w_T_b, cam_b );
    RosMarkerUtils::setcolor_to_marker( .0, 1.0, 0.0, cam_b );

    RosMarkerUtils::init_camera_marker( cam_b_new, 3.0 );
    cam_b_new.ns = "camera";
    cam_b_new.id = 2;
    RosMarkerUtils::setpose_to_marker( w_Tcap_b, cam_b_new );
    RosMarkerUtils::setcolor_to_marker( 0.0, 0.0, 1.0, cam_b_new );


    while( ros::ok() )
    {
        chatter_pub.publish( cam_a );
        chatter_pub.publish( cam_b );
        chatter_pub.publish( cam_b_new );
        ros::spinOnce();
        loop_rate.sleep();
    }





}
