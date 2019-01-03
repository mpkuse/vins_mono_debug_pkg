/*
    Ransac with theia. First will try the example on official website.
    if that works well will try to use RanSAC with DlsPnp()
*/

#include <iostream>

#include <theia/theia.h>
#include <theia/solvers/ransac.h>
// #include <theia/solvers/random.h>
#include <theia/solvers/estimator.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;


#include "../utils/PoseManipUtils.h"
#include "../utils/TermColor.h"

#if 0
// Our "data".
struct Point {
  double x; double y;
};

// Our "model".
struct Line {
  double m; double b;
};

// Estimator class.
class LineEstimator: public theia::Estimator<Point, Line> {
public:
  // Number of points needed to estimate a line.
  double SampleSize() const  { return 2; }

  // Estimate a line from two points.
  bool EstimateModel(const std::vector<Point>& data,
                     std::vector<Line>* models) const {
    Line model;
    model.m = (data[1].y - data[0].y)/(data[1].x - data[0].x);
    model.b = data[1].y - model.m*data[1].x;
    models->push_back(model);
    return true;
  }

  // Calculate the error as the y distance of the point to the line.
  double Error(const Point& point, const Line& line) const {
    return point.y - (line.m*point.x + line.b);
  }
};


int main (int argc, char** argv) {

  std::random_device rd;
  std::mt19937 e2(rd());
  std::uniform_real_distribution<> dist(0, 10);
  std::uniform_real_distribution<> dist2(0, 1);




  // We put pseudo-code here for simplicity.
  std::vector<Point> input_data;

  // Add 700 inliers.
  for (int i = 0; i < 700; i++) {
      Point inlier_point;
      inlier_point.x = dist(e2);
      inlier_point.y = 2*inlier_point.x + 8 + drand48() ;
    input_data.push_back(inlier_point);
  }
  // Add 300 outliers.
  for (int i = 0; i < 300; i++) {
      Point outlier_point;
      outlier_point.x = dist(e2);
      outlier_point.y = dist(e2);
    input_data.push_back(outlier_point);
  }

  // Specify RANSAC parameters.
  double error_threshold = 0.3;
  int min_num_inliers = 600;
  int max_iters = 1000;

  // Estimate the line with RANSAC.
  LineEstimator line_estimator;
  Line best_line;
  // Set the ransac parameters.
  theia::RansacParameters params;
  params.error_thresh = 0.1;

  // Create Ransac object, specifying the number of points to sample to
  // generate a model estimation.
  theia::Ransac<LineEstimator> ransac_estimator(params, line_estimator);
  // Initialize must always be called!
  ransac_estimator.Initialize();

  theia::RansacSummary summary;
  ransac_estimator.Estimate(input_data, &best_line, &summary);
  cout << "Line y = " << best_line.m << "*x + " << best_line.b;

  return 0;
}


#endif


/////////// DlsPnp-RANSAC ///////////////////
// Data
struct CorrespondencePair_3d2d {
    Vector3d a_X; // 3d point expressed in co-ordinate system of `image-a`
    Vector2d uv_d; //feature point in normalized-image co-ordinates. of the other view (b)
};

// Model
struct RelativePose {
    Matrix4d b_T_a;
};


class DlsPnpWithRansac: public theia::Estimator<CorrespondencePair_3d2d, RelativePose> {
public:
    // Number of points needed to estimate a line.
    double SampleSize() const  { return 50; }

    // Estimate a pose from N points
    bool EstimateModel( const std::vector<CorrespondencePair_3d2d>& data,
                            std::vector<RelativePose>* models) const {

        std::vector<Vector2d> feature_position;
        std::vector<Vector3d> world_point;
        for( int i=0 ; i<data.size() ; i++ ) {
            feature_position.push_back( data[i].uv_d );
            world_point.push_back( data[i].a_X );
        }

        std::vector<Quaterniond> solution_rotations;
        std::vector<Vector3d> solution_translations;

        theia::DlsPnp( feature_position, world_point, &solution_rotations, &solution_translations );
        if( solution_rotations.size() == 1 ) {
            RelativePose r;
            r.b_T_a = Matrix4d::Identity();
            r.b_T_a.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
            r.b_T_a.col(3).topRows(3) = solution_translations[0];
            models->push_back( r );
        }
        else {
            return false;
        }
    }


    double Error( const CorrespondencePair_3d2d& point, const RelativePose& r ) const {
        Vector3d  b_X = r.b_T_a.topLeftCorner(3,3) * point.a_X + r.b_T_a.col(3).topRows(3);
        b_X /= b_X(2);
        return abs(b_X(0) - point.uv_d(0)) + abs(b_X(1) - point.uv_d(1));
    }
};
/////////// END DlsPnp-RANSAC ///////////////////

int main()
{

}
