#include "CameraGeometry.h"

MonoGeometry::MonoGeometry(camodocal::CameraPtr _camera)
{
    assert( _camera && "Abstract Camera is not set. You need to init camera before setting it to geometry clases" );
    if( !_camera ) {
        cout << "Abstract Camera is not set. You need to init camera before setting it to geometry clases\n";
        exit(10);
    }
    this->camera = _camera;

    // set K_new as default
    GeometryUtils::getK( this->camera, this->K_new );
    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;

    // make rectification maps. Remember to remake the maps when set_K is called
    this->camera->initUndistortRectifyMap( map_x, map_y, new_fx, new_fy, cv::Size(0,0), new_cx, new_cy );
}


void MonoGeometry::set_K( Matrix3d K )
{
    std::lock_guard<std::mutex> lk(m);
    this->K_new = K;
    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;

    // once a new K is set will have to recompute maps.
    this->camera->initUndistortRectifyMap( map_x, map_y,
                                    new_fx, new_fy, cv::Size(0,0),
                                    new_cx, new_cy );
}

void MonoGeometry::do_image_undistortion( const cv::Mat& im_raw, cv::Mat & im_undistorted )
{
    std::lock_guard<std::mutex> lk(m);
    cv::remap( im_raw, im_undistorted, map_x, map_y, CV_INTER_LINEAR );
}

//-------------------------------------------------------------------------------------//

StereoGeometry::StereoGeometry( camodocal::CameraPtr _left_camera,
                camodocal::CameraPtr _right_camera,
                Matrix4d __right_T_left )
{
    if( !_left_camera || !_right_camera )
    {
        cout << "ERROR : Abstract stereo Camera is not set. You need to init camera before setting it to geometry clases";
        exit(10);
    }
    assert( _left_camera && _right_camera  && "Abstract stereo Camera is not set. You need to init camera before setting it to geometry clases" );

    this->camera_left = _left_camera;
    this->camera_right = _right_camera;
    this->right_T_left = Matrix4d( __right_T_left ); //stereo extrinsic. relative pose between two pairs.

    // set K_new as left_camera's intrinsic
    GeometryUtils::getK( this->camera_left, this->K_new );
    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;

    left_geom = std::make_shared<MonoGeometry>( this->camera_left );
    right_geom = std::make_shared<MonoGeometry>( this->camera_right );
    left_geom->set_K( this->K_new );
    right_geom->set_K( this->K_new );

    // stereo rectification maps
    // theory : http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/tutorial.html


}



void StereoGeometry::set_stereoextrinsic( Matrix4d __right_T_left )
{
    std::lock_guard<std::mutex> lk(m_extrinsics);
    this->right_T_left = Matrix4d( __right_T_left );
    make_stereo_rectification_maps();
}

void StereoGeometry::set_stereoextrinsic(Vector4d quat_xyzw, Vector3d tr_xyz )
{
    std::lock_guard<std::mutex> lk(m_extrinsics);
    Matrix4d transform ;  //right_T_left
    PoseManipUtils::raw_xyzw_to_eigenmat( quat_xyzw, tr_xyz, transform );
    this->right_T_left = Matrix4d( transform );
    make_stereo_rectification_maps();
}

const Matrix4d& StereoGeometry::get_stereoextrinsic()
{
    std::lock_guard<std::mutex> lk(m_extrinsics);
    return this->right_T_left;
}

void StereoGeometry::fundamentalmatrix_from_stereoextrinsic( Matrix3d& F )
{
    Matrix3d Tx, _Rot;
    {
    std::lock_guard<std::mutex> lk2(m_extrinsics);
    PoseManipUtils::vec_to_cross_matrix( right_T_left.col(3).topRows(3), Tx );
    _Rot = right_T_left.topLeftCorner(3,3);
    }

    // The actual formula is : inverse(K_right)' * Tx * R * inverse(K_left)
    // but we use the same K_new for both. This is a subtle point here.
    F = this->get_K().transpose().inverse() * Tx * _Rot * this->get_K().inverse();  //< Fundamental Matrix
}


void StereoGeometry::draw_epipolarlines( cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted )
{
    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

    // if image is 1 channel make it to 3 channel so that I can plot colored lines and points.
    cv::Mat imleft_undistorted_3chnl, imright_undistorted_3chnl;
    if( imleft_undistorted.channels() == 1 )
        cv::cvtColor(imleft_undistorted, imleft_undistorted_3chnl, CV_GRAY2BGR );
    else
        imleft_undistorted_3chnl = imleft_undistorted;

    if( imright_undistorted.channels() == 1 )
        cv::cvtColor(imright_undistorted, imright_undistorted_3chnl, CV_GRAY2BGR);
    else
        imright_undistorted_3chnl = imright_undistorted;

    imleft_undistorted = imleft_undistorted_3chnl;
    imright_undistorted = imright_undistorted_3chnl;


    // Fundamental Matrix
    Matrix3d F;
    this->fundamentalmatrix_from_stereoextrinsic( F );
    cout << "[StereoGeometry::draw_epipolarlines]F=" << F.format(numpyFmt) << endl;



    // tryout multiple points and get their corresponding epipolar line.
    for( int i=0 ; i<500; i+=20 ) {
    #if 1
    // take a sample point x on left image and find the corresponding line on the right image
    Vector3d x(1.5*i, i, 1.0);
    Vector3d ld = F * x;
    MiscUtils::draw_point( x, imleft_undistorted, cv::Scalar(255,0,0) );
    MiscUtils::draw_line( ld, imright_undistorted, cv::Scalar(255,0,0) );
    #endif

    #if 1
    // take a sample point on right image and find the corresponding line on the left image
    Vector3d xd(i, i, 1.0);
    Vector3d l = F.transpose() * xd;
    MiscUtils::draw_line( l, imleft_undistorted, cv::Scalar(0,0,255) );
    MiscUtils::draw_point( xd, imright_undistorted, cv::Scalar(0,0,255) );
    #endif
    }

}


void StereoGeometry::draw_srectified_epipolarlines( cv::Mat& imleft_srectified, cv::Mat& imright_srectified )
{
    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

    // if image is 1 channel make it to 3 channel so that I can plot colored lines and points.
    cv::Mat imleft_srectified_3chnl, imright_srectified_3chnl;
    if( imleft_srectified.channels() == 1 )
        cv::cvtColor(imleft_srectified, imleft_srectified_3chnl, CV_GRAY2BGR );
    else
        imleft_srectified_3chnl = imleft_srectified;

    if( imright_srectified.channels() == 1 )
        cv::cvtColor(imright_srectified, imright_srectified_3chnl, CV_GRAY2BGR);
    else
        imright_srectified_3chnl = imright_srectified;

    imleft_srectified = imleft_srectified_3chnl;
    imright_srectified = imright_srectified_3chnl;




    Matrix3d F = rm_fundamental_matrix; //< This was computed by make_stereo_rectification_maps.
    // this->fundamentalmatrix_from_stereoextrinsic( F );
    cout << "[StereoGeometry::draw_srectified_epipolarlines]F=" << F.format(numpyFmt) << endl;



    // tryout multiple points and get their corresponding epipolar line.
    for( int i=0 ; i<500; i+=20 ) {
    #if 1
    // take a sample point x on left image and find the corresponding line on the right image
    Vector3d x(1.5*i, i, 1.0);
    Vector3d ld = F * x;
    MiscUtils::draw_point( x, imleft_srectified, cv::Scalar(255,0,0) );
    MiscUtils::draw_line( ld, imright_srectified, cv::Scalar(255,0,0) );
    #endif

    #if 1
    // take a sample point on right image and find the corresponding line on the left image
    Vector3d xd(i, i, 1.0);
    Vector3d l = F.transpose() * xd;
    MiscUtils::draw_line( l, imleft_srectified, cv::Scalar(0,0,255) );
    MiscUtils::draw_point( xd, imright_srectified, cv::Scalar(0,0,255) );
    #endif
    }

}


void StereoGeometry::set_K( Matrix3d K )
{
    std::lock_guard<std::mutex> lk(m_intrinsics);

    this->K_new = K;
    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;
    make_stereo_rectification_maps();
}

void StereoGeometry::set_K( float _fx, float _fy, float _cx, float _cy )
{
    std::lock_guard<std::mutex> lk(m_intrinsics);
    this->K_new << _fx, 0.0, _cx,
                    0.0, _fy, _cy,
                    0.0, 0.0, 1.0;

    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;
    make_stereo_rectification_maps();
}

const Matrix3d& StereoGeometry::get_K()
{
    std::lock_guard<std::mutex> lk(m_intrinsics);
    return this->K_new;
}


void StereoGeometry::do_image_undistortion( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted
                        )
{
    std::lock_guard<std::mutex> lk(m_intrinsics);
    cout << "imleft_raw " << MiscUtils::cvmat_info( imleft_raw ) << endl;
    cout << "imright_raw " << MiscUtils::cvmat_info( imright_raw ) << endl;

    // this doesn't need a lock as this is not dependent on
    left_geom->do_image_undistortion( imleft_raw, imleft_undistorted );
    right_geom->do_image_undistortion( imright_raw, imright_undistorted );
}

void StereoGeometry::make_stereo_rectification_maps()
{
    cv::Size imsize = cv::Size( camera_left->imageWidth(), camera_left->imageHeight());

    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");


    // Adopted from : http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/node18.html
    cout << "[compute_stereo_rectification_transform]" << endl;





    Matrix3d right_R_left = this->right_T_left.topLeftCorner(3,3);
    Vector3d right_t_left = this->right_T_left.col(3).topRows(3);
    cv::Mat R, T;
    cv::eigen2cv( right_R_left, R );
    cv::eigen2cv( right_t_left, T );


    cv::Mat K_new_cvmat;
    cv::eigen2cv( this->K_new, K_new_cvmat );

    // TODO: If need be write getters for these matrices.
    // cv::Mat R1, R2;
    // cv::Mat P1, P2;
    // cv::Mat Q;

    cv::Mat D;
    // !! Main Call OpenCV !! //
    /// The below function does exactly this: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/node18.html
    // See OpenCVdoc: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6
    cv::stereoRectify( K_new_cvmat, D, K_new_cvmat, D, imsize, R, T,
                        rm_R1, rm_R2, rm_P1, rm_P2, rm_Q
                    );

    cout << "\tK_new=" << cv::format(K_new_cvmat, cv::Formatter::FMT_NUMPY)  << endl;
    cout << "\tR1=" << cv::format(rm_R1, cv::Formatter::FMT_NUMPY) << endl;
    cout << "\tR2=" << cv::format(rm_R2, cv::Formatter::FMT_NUMPY) << endl;
    cout << "\tP1=" << cv::format(rm_P1, cv::Formatter::FMT_NUMPY) << endl;
    cout << "\tP2=" << cv::format(rm_P2, cv::Formatter::FMT_NUMPY) << endl;
    // cout << TermColor::RESET();

    // For horizontal stereo :
    // P2(0,3) := Tx * f, where f = P2(0,0) and Tx is the position of left camera as seen by the right camera. right_T_left.
    double Tx = rm_P2.at<double>(0,3) / rm_P2.at<double>(0,0);
    rm_shift = Vector3d( Tx, 0, 0);
    cout << "\tTx = " << Tx <<  " Tx is the position of left camera as seen by the right camera. right_T_left" << endl;
    Matrix3d TTx;
    PoseManipUtils::vec_to_cross_matrix( rm_shift, TTx );
    rm_fundamental_matrix = K_new.transpose().inverse() * TTx  * K_new.inverse();
    cout << "F_rect=" << rm_fundamental_matrix.format(numpyFmt) << endl;


    // For vertical stereo :
    // P2(1,3) := Ty * f, where f = P2(0,0) and Tx is the horizontal shift between cameras
    // double Ty = P2(1,3) / P2(0,0);

    cout << "[/compute_stereo_rectification_transform]" << endl;



    // !! Make Stereo Rectification Maps !! //
    cout << "[make_rectification_maps]\n";
    cv::initUndistortRectifyMap( K_new_cvmat, D, rm_R1, rm_P1, imsize, CV_32FC1, map1_x, map1_y );
    cv::initUndistortRectifyMap( K_new_cvmat, D, rm_R2, rm_P2, imsize, CV_32FC1, map2_x, map2_y );
    cout << "\tmap1_x: " << MiscUtils::cvmat_info( map1_x ) << endl;
    cout << "\tmap1_y: " << MiscUtils::cvmat_info( map1_y ) << endl;
    cout << "\tmap2_x: " << MiscUtils::cvmat_info( map2_x ) << endl;
    cout << "\tmap2_y: " << MiscUtils::cvmat_info( map2_y ) << endl;
    cout << "[/make_rectification_maps]\n";

}

void StereoGeometry::do_stereo_rectification_of_undistorted_images(
    const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
    cv::Mat& imleft_srectified, cv::Mat& imright_srectified )
{
    cv::remap( imleft_undistorted, imleft_srectified, this->map1_x, this->map1_y, CV_INTER_LINEAR );
    cv::remap( imright_undistorted, imright_srectified, this->map2_x, this->map2_y, CV_INTER_LINEAR );
}



void StereoGeometry::do_stereo_rectification_of_raw_images(
        const cv::Mat imleft_raw, const cv::Mat imright_raw,
        cv::Mat& imleft_srectified, cv::Mat& imright_srectified )
{
    cv::Mat imleft_undistorted, imright_undistorted;

    // raw --> undistorted
    do_image_undistortion(imleft_raw, imright_raw,
        imleft_undistorted, imright_undistorted
        );

    // undistorted --> stereo rectify
    do_stereo_rectification_of_undistorted_images(imleft_undistorted, imright_undistorted,
        imleft_srectified, imright_srectified
    );
}

//-------------------------------------------------------------------------------------//
void GeometryUtils::make_K( float new_fx, float new_fy, float new_cx, float new_cy, Matrix3d& K )
{
    K << new_fx, 0, new_cx,
              0, new_fy, new_cy,
              0, 0, 1;
}





void GeometryUtils::getK( camodocal::CameraPtr m_cam, Matrix3d& K )
{
    Matrix3d K_rect;
    vector<double> m_camera_params;
    m_cam->writeParameters( m_camera_params ); // retrive camera params from Abstract Camera.
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
        case camodocal::Camera::ModelType::KANNALA_BRANDT:
            K_rect << m_camera_params[4], 0, m_camera_params[6],
                      0, m_camera_params[5], m_camera_params[7],
                      0, 0, 1;
            break;
            default:
            // TODO: Implement for other models. Look at initUndistortRectifyMap for each of the abstract class.
            cout << "[getK] Wrong\nQuit....";
            exit(10);

    }
    K = Matrix3d(K_rect);
}
