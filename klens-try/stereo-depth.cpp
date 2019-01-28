
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
#include "../utils/RawFileIO.h"


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

void load_extrinsic( const string fname, Vector4d& q_xyzw, Vector3d& tr_xyz )
{
// Open fullpath of extrinsic.yaml
        cout << "opencv yaml reading: open file: " << fname << endl;
        cv::FileStorage fs(fname, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            cout << "config_file asked to open extrinsicbasline file but it cannot be opened.\nTHIS IS FATAL, QUITING" << endl;;
            exit(1);
        }

        cout << TermColor::GREEN() << "successfully opened file "<< fname << TermColor::RESET() << endl;
        cv::FileNode n = fs["transform"];
        // Vector4d q_xyzw;
        q_xyzw << (double)n["q_x"] , (double)n["q_y"] ,(double) n["q_z"] ,(double) n["q_w"];

        // Vector3d tr_xyz;
        tr_xyz << (double)n["t_x"] , (double)n["t_y"] ,(double) n["t_z"];

        cout << "--values from file--\n" << TermColor::iGREEN();
        cout << "q_xyzw:\n" << q_xyzw << endl;
        cout << "tr_xyz:\n" << tr_xyz << endl;
        cout << TermColor::RESET() << endl;
}


int main() {
    const std::string BASE = "/Bulk_Data/klens_gmbh/0_2_80/";

    //--------- Intrinsics load
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/camera_left.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/camera_right.yaml");
    // camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_left.yaml");
    // camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_right.yaml");

    cout << left_camera->parametersToString() << endl;
    cout << right_camera->parametersToString() << endl;


    //----------- Stereo Base line load (also called extrinsic calibration)
        // mynt eye
    Vector4d q_wxyz;// = Vector4d( -1.8252509868889259e-04,-1.6291774489779708e-03,-1.2462127842978489e-03,9.9999787970731446e-01 );
    Vector3d tr_xyz;// = Vector3d( -1.2075905420832895e+02/1000.,5.4110610639412482e-01/1000.,2.4484815673909591e-01/1000. );
    load_extrinsic( BASE+"/extrinsics.yaml", q_wxyz, tr_xyz );
    tr_xyz *= 1000.;
    tr_xyz(2) = 0.;

    Matrix4d right_T_left;
    PoseManipUtils::raw_xyzw_to_eigenmat( q_wxyz, tr_xyz, right_T_left );
    // cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;


    //-------------------- init stereogeom
    std::shared_ptr<StereoGeometry> stereogeom;
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );

    //---------------------- READ image
    cv::Mat imleft_raw = cv::imread( BASE+"/left.jpg",0);
    cv::Mat imright_raw =cv::imread( BASE+"/right.jpg",0);

    //----------------- Actual computations are only these:
    MatrixXd _3dpts; //4xN
    cv::Mat imleft_srectified, imright_srectified;
    cv::Mat disparity_for_visualization;

    stereogeom->get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(imleft_raw, imright_raw,
        imleft_srectified, imright_srectified,
         _3dpts, disparity_for_visualization );

   cv::imshow( "disparity_for_visualization", disparity_for_visualization );
   cv::imwrite( BASE+"/disparity_for_visualization.png",disparity_for_visualization );
   cv::waitKey(0);

}
