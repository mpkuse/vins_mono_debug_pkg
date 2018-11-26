#pragma once
// This has 2 classes
// A) MonoGeometry
// B) StereoGeometry

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>


#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "MiscUtils.h"
#include "ElapsedTime.h"
#include "PoseManipUtils.h"
#include "TermColor.h"

// Threads and threadsafety
#include <thread>
#include <mutex>
#include <atomic>

class MonoGeometry {
public:
    MonoGeometry() {  }

    MonoGeometry(camodocal::CameraPtr _camera );

    // Set the new intrinsic matrix (optional). By default will use the same as
    // the abstract camera. but the K matrix is essentially just scaling pixel sizes.
    // this comes to play in stereopairs and/or motion stereo.
    void set_K( Matrix3d K );

    // this is just a call to cv::remap with the maps already computed.
    void do_image_undistortion( const cv::Mat& im_raw, cv::Mat & im_undistorted );

private:
    camodocal::CameraPtr camera;
    Matrix3d K_new;
    float new_fx, new_fy, new_cx, new_cy;

    cv::Mat map_x, map_y;

    std::mutex m;

};


// There are 3 types of images
//      im*_raw
//      im*_undistorted
//      im*_stereo_rectified
class StereoGeometry {
public:
    // right_T_left: extrinsic calibration for stereo pair. The position of left camera as viewed from right camera.
    StereoGeometry( camodocal::CameraPtr _left_camera,
                    camodocal::CameraPtr _right_camera,
                    Matrix4d right_T_left
                );


    // intrinsic cam matrix for both cameras. Here it is important to set this
    // else the block matching computation won't be valid. If this is not set
    // we will use the matrix for left camera as K_new.
    // Everytime a new K is set, we have to recompute the maps.
    void set_K( Matrix3d K );
    void set_K( float _fx, float _fy, float _cx, float _cy );
    const Matrix3d& get_K();

    // set extrinsic. THis is thread safe
    // Everytime a new extrinsic is set, we have to recompute the maps. 
    void set_stereoextrinsic( Matrix4d __right_T_left );
    void set_stereoextrinsic( Vector4d quat_xyzw, Vector3d tr_xyz );
    const Matrix4d& get_stereoextrinsic();

    void fundamentalmatrix_from_stereoextrinsic( Matrix3d& F );

    // Draw epipolar lines on both views. These images will be overwritten. You need
    // to give me undistorted image-pairs. You can undistory image pair with
    // StereoGeometry::do_image_undistortion()
    void draw_epipolarlines( cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted );
    void draw_srectified_epipolarlines( cv::Mat& imleft_srectified, cv::Mat& imright_srectified );


    void do_image_undistortion( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted
                            );

    // given the undistorted images outputs the stereo-rectified pairs
    // This is essentially just cv::remap using the stereo-rectification maps
    void do_stereo_rectification_of_undistorted_images(
        const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
        cv::Mat& imleft_srectified, cv::Mat& imright_srectified );


    // Given raw image pair return stereo-rectified pair. This is a 2 step process
    // raw --> undistorted --> stereo rectified
    void do_stereo_rectification_of_raw_images(
        const cv::Mat imleft_raw, const cv::Mat imright_raw,
        cv::Mat& imleft_srectified, cv::Mat& imright_srectified );


private:
    camodocal::CameraPtr camera_left, camera_right;
    Matrix4d right_T_left; ///< extrinsic calibration of the stereopair.

    Matrix3d K_new;
    float new_fx, new_fy, new_cx, new_cy;

    std::shared_ptr<MonoGeometry> left_geom, right_geom;

    // stereo rectify maps
    // theory : http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/tutorial.html

    std::mutex m_extrinsics;
    std::mutex m_intrinsics;

    // This function is called everytime you change the intrinsics and extrinsics for
    // the stereo pair. This outputs the stereo-rectification maps, ie. map1_x, map1_y, map2_x, map2_y
    // This is not intended to be a public call.
    // TODO: thread safety
    void make_stereo_rectification_maps();
    cv::Mat rm_R1, rm_R2, rm_P1, rm_P2, rm_Q;
    Vector3d rm_shift; // right_T_left
    Matrix3d rm_fundamental_matrix; // fundamental matrix after rectification
    cv::Mat map1_x, map1_y, map2_x, map2_y;


};

class GeometryUtils {
public:
    // Given the focal length make a K out of it
    static void make_K( float new_fx, float new_fy, float new_cx, float new_cy, Matrix3d& K );

    // For a camera gets a K
    static void getK( camodocal::CameraPtr m_cam, Matrix3d& K );

};
