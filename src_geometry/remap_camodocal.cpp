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

#include "../utils/MiscUtils.h"
#include "../utils/ElapsedTime.h"
#include "../utils/PoseManipUtils.h"

// For a camera gets a K
void getK( camodocal::CameraPtr m_cam, Matrix3d& K )
{
    Matrix3d K_rect;
    vector<double> m_camera_params;
    m_cam->writeParameters( m_camera_params );
    // camodocal::CataCamera::Parameters p();
    // cout << "size=" << m_camera_params.size() << " ::>\n" ;
    // for( int i=0 ; i<m_camera_params.size() ; i++ ) cout << "\t" << i << " " << m_camera_params[i] << endl;

    switch( m_cam->modelType() )
    {
        case camodocal::Camera::ModelType::MEI:
            K_rect << m_camera_params[5], 0, m_camera_params[7],
                      0, m_camera_params[6], m_camera_params[8],
                      0, 0, 1;
            break;
        case camodocal::Camera::ModelType::PINHOLE:
            K_rect << m_camera_params[4], 0, m_camera_params[6],
                      0, m_camera_params[5], m_camera_params[7],
                      0, 0, 1;
            break;
            default:
            // TODO: Implement for other models. Look at initUndistortRectifyMap for each of the abstract class.
            cout << "Wrong\nQuit....";
            exit(10);

    }
    K = Matrix3d(K_rect);
}


// new K
void getK( float new_fx, float new_fy, float new_cx, float new_cy, Matrix3d& K )
{
    // TODO: Pass these as arguments
    // float new_fx = 375.;
    // float new_fy = 375.;
    // float new_cx = 376.;
    // float new_cy = 240.;

    K << new_fx, 0, new_cx,
              0, new_fy, new_cy,
              0, 0, 1;

}

void toCross( const Vector3d& t, Matrix3d& Tx )
{
    // Tx = Matrix3d::Zero();
    Tx << 0, -t(2) , t(1) ,
          t(2), 0,  -t(0),
          -t(1), t(0), 0;

}

void do_image_undistortion( camodocal::CameraPtr m_cam, Matrix3d new_K, const cv::Mat& im_raw, cv::Mat & im_undistorted )
{
    float new_fx = new_K(0,0); // 375.;
    float new_fy = new_K(1,1); //375.;
    float new_cx = new_K(0,2); //376.;
    float new_cy = new_K(1,2); //240.;

    cv::Mat map_x, map_y;
    m_cam->initUndistortRectifyMap( map_x, map_y, new_fx, new_fy, cv::Size(0,0), new_cx, new_cy ); // inefficient. but i don't care now.TODO ideally should create the map only once and reuse it.
    cv::remap( im_raw, im_undistorted, map_x, map_y, CV_INTER_LINEAR );
}


double Slope(int x0, int y0, int x1, int y1){
     return (double)(y1-y0)/(x1-x0);
}

void fullLine(cv::Mat& img, cv::Point2f a, cv::Point2f b, cv::Scalar color){
     double slope = Slope(a.x, a.y, b.x, b.y);

     cv::Point2f p(0,0), q(img.cols,img.rows);

     p.y = -(a.x - p.x) * slope + a.y;
     q.y = -(b.x - q.x) * slope + b.y;

     cv::line(img,p,q,color,1,8,0);
}

// draw line on the image. l = (a,b,c) for ax+by+c = 0
void draw_line( const Vector3d l, cv::Mat& im, cv::Scalar color )
{
    // C++: void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    cv::Point2f a(0.0, -l(2)/l(1) );
    cv::Point2f b(-l(2)/l(0), 0.0 );
    // cout << a << "<--->" << b << endl;
    // cv::line( im, a, b, cv::Scalar(255,255,255) );
    fullLine( im, a, b, color );
}

void draw_point( const Vector3d pt, cv::Mat& im, cv::Scalar color  )
{
    // C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    cv::circle( im, cv::Point2f( pt(0)/pt(2), pt(1)/pt(2) ), 2, color, -1   );

}

void draw_point( const Vector2d pt, cv::Mat& im, cv::Scalar color  )
{
    cv::circle( im, cv::Point2f( pt(0), pt(1) ), 2, color, -1   );
}

// Trying out Epipolar Geometry with camodocal.
// Theory Background : http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/tutorial.html
// Note: OpenCV's functions are extremely confusing and best not used. These things are actually a lot
// simpler than seem to be by looking at opencv's function.
//
// Here is what I learnt:
// - Given raw images undistort the images. When you undistort you can set the camera intrinsic new to arbitary values. See my K_new for example.
// - After undisort, you can as well forget about camera model and just use K_new
// - For stereo pair you want to set the same K_new for both left and right image. K_new is essentially just scaling the normalized co-ordinates to pixels.
// - Fundamental matrix from given R,T. F = K_new.transpose().inverse() * Tx * right_T_left.topLeftCorner(3,3) * K_new.inverse();
// - Epipoles are the nullspaces of matrix F and F.transpose().
// - x <---> ld=Fx and l=F.transpose() x <---> xd.
int main()
{
    // const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_multi_loops_in_lab/";
    const std::string BASE = "/Bulk_Data/ros_bags/bluefox_stereo/calib/leveled_cam_sampled/";
    // const std::string BASE = "/Bulk_Data/ros_bags/bluefox_stereo/calib/right_titled_sampled/";

    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");



    camodocal::CameraPtr m_camera_left, m_camera_right;
    // m_camera_left = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    // m_camera_right = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");
    // m_camera_left = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string("/app/catkin_ws/src/vins_mono_debug_pkg/build/stereo_calib2_pinhole")+"/camera_left.yaml");
    // m_camera_right = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string("/app/catkin_ws/src/vins_mono_debug_pkg/build/stereo_calib2_pinhole")+"/camera_right.yaml");
    m_camera_left = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_left.yaml");
    m_camera_right = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_right.yaml");
    // m_camera_left = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../right_titled_pinhole/")+"/camera_left.yaml");
    // m_camera_right = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../right_titled_pinhole/")+"/camera_right.yaml");

    cout << m_camera_left->parametersToString() << endl;
    cout << m_camera_right->parametersToString() << endl;

    //
    // Intrinsics
    // Camera intrinsics from camera calibration files. These have no significance in stereo computation. As well not get these.
    Matrix3d K, Kd;
    getK( m_camera_left, K );
    getK( m_camera_right, Kd );
    cout << "K\n"<< K << endl;
    cout << "Kd\n"<< Kd << endl;

    // I can set new intrinsic to whatever I want after undistortion.
    // This has been a major point of confusion. Note that K_new just scales the normalized image co-ordinates (irrespective of camera model) to pixels.
    // In case of a stereo pair you want both your camera to have the same scaling.
    Matrix3d K_new;
    getK( 375.0, 375.0, 376.0, 240.0, K_new );
    cout << "K_new\n"<< K_new << endl;

    //
    // Extrinsic (Baseline)
    Matrix4d right_T_left = Matrix4d::Zero();
    // from mynt
    // double _rot_xyzw[] = {-1.8252509868889259e-04,-1.6291774489779708e-03,-1.2462127842978489e-03,9.9999787970731446e-01};
    // double _tr_xyz[] = {-1.2075905420832895e+02/1000.,5.4110610639412482e-01/1000.,2.4484815673909591e-01/1000.};

    // from bluefox_stereo leveled
    double _rot_xyzw[] = { -1.7809713490350254e-03, 4.2143149583451564e-04,4.1936662160154632e-02, 9.9911859501433165e-01};
    double _tr_xyz[] = {-1.4031938291177164e+02/1000.,-6.6214729932530441e+00/1000.,1.4808567571722902e+00/1000.};

    // from bluefox_stereo right-titled
    // double _rot_xyzw[] = {7.6402817795312578e-02, -2.5540621037530589e-02,-3.6647229516532570e-01, 9.2693491841995723e-01};
    // double _tr_xyz[] = { -1.1921506144031111e+01/1000.,6.3120089648277940e+01/1000.,-1.8124558314119258e+02/1000.};

    PoseManipUtils::raw_xyzw_to_eigenmat( _rot_xyzw, _tr_xyz, right_T_left );
    cout << "right_T_left:" << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;

    //
    // Load raw image
    int frame_id = 20;
    // cv::Mat im_left =  cv::imread( BASE+"/"+std::to_string(frame_id)+".jpg" );
    // cv::Mat im_right = cv::imread( BASE+"/"+ std::to_string(frame_id)+"_1.jpg" );
    cv::Mat im_left =  cv::imread( BASE+"/cam0_"+std::to_string(frame_id)+".png" );
    cv::Mat im_right = cv::imread( BASE+"/cam1_"+ std::to_string(frame_id)+".png" );

    cv::Mat im_left_undistorted, im_right_undistorted;
    do_image_undistortion( m_camera_left, K_new, im_left, im_left_undistorted );
    do_image_undistortion( m_camera_right, K_new, im_right, im_right_undistorted );



    // cout << "right_t_left: " << right_T_left.col(3).topRows(3).transpose() << endl;
    Matrix3d Tx;
    toCross( right_T_left.col(3).topRows(3), Tx );
    // cout << "Tx" << Tx << endl;

    // Matrix3d F = Kd.transpose().inverse() * Tx * right_T_left.topLeftCorner(3,3) * K.inverse();  //< Fundamental Matrix
    Matrix3d F = K_new.transpose().inverse() * Tx * right_T_left.topLeftCorner(3,3) * K_new.inverse();  //< Fundamental Matrix
    cout << "F="  << F.format(numpyFmt) << endl;


    // TODO : try liftProjective to get image cord in normalized co-ordinates. will scale the line back to get it in image cords.
    // Vector3d x(60, 80, 1.0);
    for( int i=0 ; i<500; i+=10 ) {
    #if 1
    // take a sample point x on left image and find the corresponding line on the right image
    Vector3d x(1.5*i, i, 1.0);
    Vector3d ld = F * x;
    draw_point( x, im_left_undistorted, cv::Scalar(255,0,255) );
    draw_line( ld, im_right_undistorted, cv::Scalar(255,0,255) );
    #endif

    #if 1
    // take a sample point on right image and find the corresponding line on the left image
    Vector3d xd(i, i, 1.0);
    Vector3d l = F.transpose() * xd;
    draw_line( l, im_left_undistorted, cv::Scalar(0,0,255) );
    draw_point( xd, im_right_undistorted, cv::Scalar(0,0,255) );
    #endif


    cv::imshow( "left", im_left );
    cv::imshow( "right", im_right );
    cv::imshow( "im_left_undistorted", im_left_undistorted );
    cv::imshow( "im_right_undistorted", im_right_undistorted );
    cv::waitKey(0);
    }
}



// Trying out undistort
/*
int main()
{
    const std::string base = "/Bulk_Data/_tmp_cerebro/tum_magistrale1/";
    // std::string calib_file = "../examples/sample_kannala_brandt.yaml";
    // std::string calib_file = "../examples/sasmple_pinhole.yaml";
    std::string calib_file = base+"/cameraIntrinsic.yaml";


    camodocal::CameraPtr m_camera;
    std::cout << ((m_camera)?"cam is initialized":"cam is not initiazed") << std::endl; //this should print 'not initialized'

    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    std::cout << ((m_camera)?"cam is initialized":"cam is not initiazed") << std::endl; //this should print 'initialized'

    // Inbuild function for priniting
    std::cout << m_camera->parametersToString() << std::endl;

    // Custom priniting info
    std::cout << "imageWidth="  << m_camera->imageWidth() << std::endl;
    std::cout << "imageHeight=" << m_camera->imageHeight() << std::endl;
    std::cout << "model_type=" << m_camera->modelType() << std::endl;
    std::cout << "camera_name=" << m_camera->cameraName() << std::endl;


    cv::Mat map_x, map_y;
    m_camera->initUndistortRectifyMap( map_x, map_y );


    cout << "map_x, map_y: " << map_x.at<float>(50,60) << " " << map_y.at<float>(50,60) << endl;
    Vector3d outSp;
    m_camera->liftProjective( Vector2d(60.0,50.0), outSp ); // step-1: K_inv * x , step-2: apply inverse distortion ==> normalized image co-ordinates
    outSp = outSp / outSp(2);
    cout << "outSp: " << outSp << endl;

    Matrix3d K_rect;
    vector<double> m_camera_params;
    m_camera->writeParameters( m_camera_params );
    // camodocal::CataCamera::Parameters p();
    cout << "size=" << m_camera_params.size() << " ::> " << m_camera_params[0] << " " << m_camera_params[1] << endl;
    K_rect << m_camera_params[4], 0, 512. / 2,
              0, m_camera_params[5], 512. / 2,
              0, 0, 1;


    // Vector3d outSp__1 = K_rect.inverse() * outSp ;
    Vector3d outSp__1 = K_rect.inverse() * Vector3d( 60,50,1.) ;
    Vector2d outSp__2;
    m_camera->spaceToPlane( outSp__1, outSp__2  );
    // cout << K_rect.inverse() * outSp << endl;
    cout << "outSp__2: " << outSp__2 << endl; // this is exactly equal to map_x, map_y at that point.


    Vector2d __f;
    m_camera->spaceToPlane( outSp , __f );
    cout << "__f:" <<  __f << endl;

    return 0;



    // cv::remap
    for( int i=0 ; i<1000 ; i++ ) {
    // cv::Mat im = cv::imread( base+"100.jpg");
    cv::Mat im = cv::imread( base+std::to_string(i)+".jpg");
    cv::imshow( "org", im );
    // cv::waitKey(0);


    std::cout << MiscUtils::cvmat_info( map_x ) << std::endl;

    ElapsedTime timer;
    timer.tic();

    cv::Mat im_undist;
    cv::remap( im, im_undist, map_x, map_y, CV_INTER_LINEAR );
    std::cout << "im_undist : "<< MiscUtils::cvmat_info( im_undist ) << std::endl;

    std::cout << "cv::remap done in (ms) : " << timer.toc_milli() << endl;

    cv::imshow( "im_undist", im_undist  );
    cv::waitKey(0);
    }

}

*/
