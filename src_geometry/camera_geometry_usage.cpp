// Sample usage for class CameraGeometry.h/MonoGeometry and CameraGeometry.h/StereoGeometry
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
#include "../utils/TermColor.h"
#include "../utils/CameraGeometry.h"

#include "gms_matcher.h"

#include <assert.h>

#include <ceres/ceres.h>
using namespace ceres;


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



// template<int PoseBlockSize>
class UnitVectorParameterization : public ceres::LocalParameterization {
public:
    UnitVectorParameterization() {}
    virtual ~UnitVectorParameterization() {}

    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const
    {
        // Compute T (tangent space)
        // tangent space is a function of x
        double m[5], n[5];
        bool statis = gram_schmidt_orthonormalization( x, m, n );
        if( !statis )
            return false;


        // x_new := x + T*[ delta[0]; delta[1] ]
        x_plus_delta[0] = x[0] + m[0]*delta[0] + n[0]*delta[1];
        x_plus_delta[1] = x[1] + m[1]*delta[0] + n[1]*delta[1];
        x_plus_delta[2] = x[2] + m[2]*delta[0] + n[2]*delta[1];
        return true;
    }

    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const
    {
        // Compute T (tangent space)
        // tangent space is a function of x
        double m[5], n[5];
        bool statis = gram_schmidt_orthonormalization( x, m, n );
        if( !statis )
            return false;

        // \del(x') / \del(a) = [  T_{0,0}; T_{1,0}  ]

        // \del(x') / \del(b) = [  T_{0,1}; T_{1,1}  ]


        // jacobian = [ concate above 2 ]
        jacobian[0] = m[0]; jacobian[3] = n[0];
        jacobian[1] = m[1]; jacobian[4] = n[1];
        jacobian[2] = m[2]; jacobian[5] = n[2];
        return true;

    }
    virtual int GlobalSize() const { return 3; } // TODO : Generalize.
    virtual int LocalSize() const { return 2; }

    // This function return 2 vectors m, n each of dimension equal to x (ie. 3) such that x,m,n are orthogonal to each other.
    const bool gram_schmidt_orthonormalization( const double * _x , double * _m, double * _n, int len=3 ) const
    {
        VectorXd xp = VectorXd::Zero( len );
        for( int i=0 ; i<len; i++ ) xp(i) = _x[i];

        xp = xp / xp.norm();

        // equation of tangent-plane : x_p * x  + y_p * y + z_p * z  = 1,
        // where (xp,yp,zp) is a known point on the sphere,. point passing through (x_p,y_p,z_p) and
        // plane normal vector direction as (x_p, y_p, z_p).

        // assert( abs(xp.norm()-1.) < 1e-7 );
        if( abs(xp(2)) < 1E-5 )
            return false;

        VectorXd m = VectorXd::Zero(len); //< a point on tangent plane.
        m << 10., -7, 1.0 -( xp(0)*10. + xp(1)*(-7.) )/xp(2); //< assuming xp(2) is non-zero, TODO ideally should divide by the maximum of {xp(0), xp(1), xp(2) }, the points will change accordingly

        VectorXd n = VectorXd::Zero(len); //< another point on tangent place.
        n << -12., 5, 1.0-( xp(0)*(-12.) + xp(1)*(5.) )/xp(2); //< assuming xp(2) is non-zero

        m = m - xp;
        m /= m.norm(); //< unit vector in the direction of m

        n = n - xp;
        n /= n.norm(); //< unit vector in the direction of n

        m = m - proj(xp, m);  // projection of xp in the direction of m
        m = m / m.norm();
        n = n - proj( xp, n ) - proj( m, n );
        n = n / n.norm();

        // m, n are the 2 basis vectors.
        // cout << "m" << m << endl;
        // cout << "n" << n << endl;
        for( int i=0 ; i<len; i++ ) {
            _m[i] = m(i);
            _n[i] = n(i);
        }
        return true;


    }

    VectorXd proj( const VectorXd& u, const VectorXd& v ) const
    {
        return u * ( u.dot(v) / u.dot( u ) );
    }


};


class YonggenResidue {
public:
  YonggenResidue( const Vector3d& Xi, const Vector3d& Xid ) : Xi(Xi), Xid(Xid)
  {
    // this->Xi = Xi;
    // this->Xid = Xid;
  }

  template <typename T>
  bool operator()( const T* const q, const T* const t , T* residual ) const {
    // Optimization variables
    Quaternion<T> eigen_q( q[0], q[1], q[2], q[3] );

    Eigen::Matrix<T,3,3> eigen_tx;
    eigen_tx << T(0.0), -t[2] , t[1] ,
                t[2], T(0.0),  -t[0],
                -t[1], t[0], T(0.0);


    Eigen::Matrix<T,3,3> essential_matrix;
    essential_matrix = eigen_tx * eigen_q.toRotationMatrix()  ;


    // Known Constant
    Eigen::Matrix<T,3,1> eigen_Xi;
    Eigen::Matrix<T,3,1> eigen_Xid;
    eigen_Xi << T(Xi(0)), T(Xi(1)), T(Xi(2));
    eigen_Xid << T(Xid(0)), T(Xid(1)), T(Xid(2));



    // Error term

    Eigen::Matrix<T,1,1> e;
    // e = eigen_Xi - (  eigen_q.toRotationMatrix() * eigen_Xid + eigen_t );
    e = eigen_Xid.transpose() * essential_matrix * eigen_Xi;
    // e = eigen_Xi.transpose() * (essential_matrix * eigen_Xid);


    residual[0] = e(0);
    // residual[1] = e(1);
    // residual[2] = e(2);

    return true;
  }



  static ceres::CostFunction* Create(const Vector3d& _Xi, const Vector3d& _Xid)
  {
    return ( new ceres::AutoDiffCostFunction<YonggenResidue,1,4,3>
      (
        new YonggenResidue(_Xi,_Xid)
      )
    );
  }

private:
  Vector3d Xi, Xid;
};


void nudge_extrinsics( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                        const Matrix3d& K_new, const Matrix4d& right_T_left, Matrix4d& optimized_right_T_left )
{
    // implements Yonggen Ling's method
    // Y. Ling and S. Shen, "High-precision online markerless stereo extrinsic calibration," 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, pp. 1771-1778.

    // minimize_{R,t} \sum_i || (f'_i)^T E f_i || , where R,t == right_T_left.
    //           a) E is the Essential matrix E := [t]_x R
    //           b) f and f' are point feature matches (f from left f from right) in normalized image co-ordinates


    //
    // Step-1 : Match point features from `imleft_undistorted, imright_undistorted`
    MatrixXd u, ud; //< 3xN, ie. image co-ordinates represented in homogeneous cord system.
    point_feature_matches( imleft_undistorted, imright_undistorted, u, ud );
    cv::Mat dst;
    MiscUtils::plot_point_pair( imleft_undistorted, u, -1, imright_undistorted, ud, -1, dst, 0 );
    cv::imshow( "gms_matches", dst );



    //
    // Step-2 : In normalized co-ordinates : inv(K_new) * f' and inv(K_inv) * f
    MatrixXd f = K_new.inverse() * u;
    MatrixXd fd = K_new.inverse() * ud;


    //
    // Step-3 : Setup CERES problem

    // 3.1 : Initial Guess
    double T_cap_q[10], T_cap_t[10];
    PoseManipUtils::eigenmat_to_raw( right_T_left, T_cap_q, T_cap_t );
    double n_norm = sqrt( T_cap_t[0]*T_cap_t[0] + T_cap_t[1]*T_cap_t[1] + T_cap_t[2]*T_cap_t[2] );
    T_cap_t[0] /= n_norm;
    T_cap_t[1] /= n_norm;
    T_cap_t[2] /= n_norm;

    cout << "CERES Inital Guess : " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;
    cout << "CERES Initial Guess n_norm: " << n_norm << endl;
    cout << "CERES Initial Guess T_cap_t: " << T_cap_t[0] << " " << T_cap_t[1] << " " << T_cap_t[2] << endl;


    // 3.2 : Error Terms
    ceres::Problem problem;
    cout << "CERES #residues : " << f.cols() << endl;
    for( int i=0 ; i<250 /*f.cols()*/ ; i++ )
    {
        int r = rand() % f.cols();
        CostFunction* cost_function = YonggenResidue::Create( f.col(r).head(3), fd.col(r).head(3) );
        problem.AddResidualBlock( cost_function, NULL, T_cap_q, T_cap_t );
    }


    // 3.3 : Local Parameterization
    ceres::LocalParameterization *quaternion_parameterization = new ceres::QuaternionParameterization;
    // ceres::LocalParameterization *hv_parameterization = new ceres::HomogeneousVectorParameterization(3);
    ceres::LocalParameterization *hv_parameterization = new UnitVectorParameterization();
    problem.SetParameterization( T_cap_q, quaternion_parameterization );
    problem.SetParameterization( T_cap_t, hv_parameterization );

    //
    // Step-4 : Solve
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    // options.minimizer_type = ceres::LINE_SEARCH;
    // options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";
    std::cout << summary.BriefReport() << "\n";


    //
    // Step-5 : Retrive Solution
    Matrix4d T_cap;
    PoseManipUtils::raw_to_eigenmat( T_cap_q, T_cap_t, T_cap );
    cout << "CERES Solution : " << PoseManipUtils::prettyprintMatrix4d( T_cap ) << endl;
    cout << "CERES Solution T_cap_t: " << T_cap_t[0] << " " << T_cap_t[1] << " " << T_cap_t[2] << endl;
    T_cap_t[0] *= n_norm;
    T_cap_t[1] *= n_norm;
    T_cap_t[2] *= n_norm;
    optimized_right_T_left = T_cap;



}

// Stereo
int stereo_demo() {
    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");
    ElapsedTime timer;

    // const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_multi_loops_in_lab/";
    const std::string BASE = "/Bulk_Data/ros_bags/bluefox_stereo/calib/leveled_cam_sampled/";


    // Abstract Camera
    // camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    // camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_left.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_right.yaml");

    cout << left_camera->parametersToString() << endl;
    cout << right_camera->parametersToString() << endl;


    // Extrinsics - quat is wxyz . translation in meters.
        // mynt eye
    // Vector4d q_wxyz = Vector4d( -1.8252509868889259e-04,-1.6291774489779708e-03,-1.2462127842978489e-03,9.9999787970731446e-01 );
    // Vector3d tr_xyz = Vector3d( -1.2075905420832895e+02/1000.,5.4110610639412482e-01/1000.,2.4484815673909591e-01/1000. );

        // bluefox stereo leveled
    Vector4d q_wxyz = Vector4d( -1.7809713490350254e-03, 4.2143149583451564e-04,4.1936662160154632e-02, 9.9911859501433165e-01 );
    Vector3d tr_xyz = Vector3d( -1.4031938291177164e+02/1000.,-6.6214729932530441e+00/1000.,1.4808567571722902e+00/1000. );

    Matrix4d right_T_left;
    PoseManipUtils::raw_xyzw_to_eigenmat( q_wxyz, tr_xyz, right_T_left );
    cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;
    cout << "right_T_left=" << right_T_left.format( numpyFmt );


    // StereoGeometry stereogeom( left_camera,right_camera,      right_T_left  );
    std::shared_ptr<StereoGeometry> stereogeom;
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );
    stereogeom->set_K( 375.0, 375.0, 376.0, 240.0 );

    // Raw Image - Image from camera
    int frame_id = 15;
    for( int frame_id=0 ; frame_id < 100 ; frame_id++ ) {
    // cv::Mat imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+".jpg", 0 );
    // cv::Mat imright_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+"_1.jpg", 0 );
    cv::Mat imleft_raw =  cv::imread( BASE+"/cam0_"+std::to_string(frame_id)+".png",0 );
    cv::Mat imright_raw = cv::imread( BASE+"/cam1_"+ std::to_string(frame_id)+".png",0 );


    // Undistort Only
    cv::Mat imleft_undistorted, imright_undistorted;
    timer.tic();
    stereogeom->do_image_undistortion( imleft_raw, imright_raw, imleft_undistorted, imright_undistorted );
    cout << timer.toc_milli() << " :(ms) stereogeom->do_image_undistortion()\n";
    cv::imshow( "imleft_raw", imleft_raw );
    cv::imshow( "imright_raw", imright_raw );
    cv::imshow( "imleft_undistorted", imleft_undistorted );
    cv::imshow( "imright_undistorted", imright_undistorted );


    // Matrix4d optimized_right_T_left;
    // nudge_extrinsics(imleft_undistorted, imright_undistorted, stereogeom->get_K(), right_T_left, optimized_right_T_left);
    // cout << "optimized_right_T_left: " << PoseManipUtils::prettyprintMatrix4d( optimized_right_T_left) << endl;
    // stereogeom->set_stereoextrinsic( optimized_right_T_left );




    // Draw Epipolar Lines
    cv::Mat dst_imleft_undistorted = imleft_undistorted.clone();
    cv::Mat dst_imright_undistorted = imright_undistorted.clone();
    stereogeom->draw_epipolarlines(dst_imleft_undistorted, dst_imright_undistorted);
    cv::imshow( "imleft_undistorted", dst_imleft_undistorted );
    cv::imshow( "imright_undistorted", dst_imright_undistorted );



    // Stereo Rectify.
    cv::Mat imleft_srectified, imright_srectified;
    timer.tic();
    stereogeom->do_stereo_rectification_of_undistorted_images(
        imleft_undistorted, imright_undistorted,
        imleft_srectified, imright_srectified );
    cout << timer.toc_milli() << " :(ms) stereogeom->do_stereo_rectification_of_undistorted_images\n";
    cv::imshow( "imleft_srectified", imleft_srectified );
    cv::imshow( "imright_srectified", imright_srectified );



    // Draw Epipolar lines on stereo rectified images
    cv::Mat dst_imleft_srectified = imleft_srectified.clone();
    cv::Mat dst_imright_srectified = imright_srectified.clone();
    stereogeom->draw_srectified_epipolarlines(  dst_imleft_srectified, dst_imright_srectified );
    cv::imshow( "dst_imleft_srectified", dst_imleft_srectified );
    cv::imshow( "dst_imright_srectified", dst_imright_srectified );




    // SGBM - TODO consider moving this to StereoGeometry class.
    // auto sgbm = cv::StereoSGBM::create(0,16,3);
    // auto sgbm = cv::StereoSGBM::create(0,16,3);
    auto sgbm = cv::StereoBM::create();
    cv::Mat disp_raw, disp8;
    timer.tic();
    sgbm->compute( imleft_srectified, imright_srectified, disp_raw ); //< this needs 1 channel images as input
    cout << timer.toc_milli() << " :(ms) sgbm->compute\n";
    cv::normalize(disp_raw, disp8, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow( "disp8", disp8 );


    // Visualize 3d point cloud from stereo
    cv::waitKey(0);
    }



}

// Monocular
int monocular_demo()
{
    const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_multi_loops_in_lab/";

    // Abstract Camera
    camodocal::CameraPtr m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    cout << m_camera->parametersToString() << endl;

    // Init Monocular Geometry
    MonoGeometry monogeom( m_camera );

    // Raw Image - Image from camera
    int frame_id = 23;
    cv::Mat im_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+".jpg" );

    cv::Mat im_undistorted;
    monogeom.do_image_undistortion( im_raw, im_undistorted );


    cv::imshow( "im_raw", im_raw );
    cv::imshow( "im_undistorted", im_undistorted );
    cv::waitKey(0);
}

int main() {
    // monocular_demo();
    stereo_demo();

    /*
    double _x[5];
    _x[0] = 0.545435;
    _x[1] = .8;
    _x[2] = .25;

    double _m[5], _n[5];
    UnitVectorParameterization parm = UnitVectorParameterization();
    parm.gram_schmidt_orthonormalization( _x, _m, _n );
    */
}
