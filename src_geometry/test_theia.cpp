#include <iostream>

#include <theia/theia.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

int main()
{
    //
    // Read Image
    theia::FloatImage my_img("/app/lena.jpg");
    std::cout << "my_img.shape" << my_img.Rows() << " " << my_img.Cols() << std::endl;
    // TODO: get a to_opencv cv::Mat.


    //
    // Keypoint detections. internally only SIFT available.
    // TODO: If there is time can integrate opencv with theia. Mainly for feature detector and descriptors.
    // will have to inherit Feature Detector class. Doable but it is a waste of my time.

    

    Vector2d x[3]; //image pts
    Vector3d X[3]; //world pts
    for( int i=0; i<3 ; i++ ) {
        x[i] = Vector2d::Random();
        X[i] = Vector3d::Random();
    }
    std::vector<Eigen::Matrix3d> solution_rotations;
    std::vector<Eigen::Vector3d> solution_translations;
    std::cout << theia::PoseFromThreePoints( x, X, &solution_rotations, &solution_translations ) << std::endl;
    std::cout << "#solutions " << solution_translations.size() << " " << solution_rotations.size();

    std::cout << "Hello theia\n";
    return 0;
}
