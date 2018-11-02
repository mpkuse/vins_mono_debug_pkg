#include "MiscUtils.h"

string MiscUtils::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


std::vector<std::string>
MiscUtils::split( std::string const& original, char separator )
{
    std::vector<std::string> results;
    std::string::const_iterator start = original.begin();
    std::string::const_iterator end = original.end();
    std::string::const_iterator next = std::find( start, end, separator );
    while ( next != end ) {
        results.push_back( std::string( start, next ) );
        start = next + 1;
        next = std::find( start, end, separator );
    }
    results.push_back( std::string( start, next ) );
    return results;
}


void MiscUtils::plot_point_sets( const cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                        const cv::Scalar& color, bool enable_keypoint_annotation, const string& msg )
{
  MatrixXf pts_set_float;
  pts_set_float = pts_set.cast<float>();

  cv::Mat pts_set_mat;
  cv::eigen2cv( pts_set_float, pts_set_mat );

  MiscUtils::plot_point_sets( im, pts_set_mat, dst, color, enable_keypoint_annotation, msg );
}

// in place manip
void MiscUtils::plot_point_sets( cv::Mat& im, const MatrixXd& pts_set,
                                        const cv::Scalar& color, bool enable_keypoint_annotation, const string& msg )
{
  MatrixXf pts_set_float;
  pts_set_float = pts_set.cast<float>();

  cv::Mat pts_set_mat;
  cv::eigen2cv( pts_set_float, pts_set_mat );

  MiscUtils::plot_point_sets( im, pts_set_mat, im, color, enable_keypoint_annotation, msg );
}

// TODO: call the next function instead of actually doing the work.
void MiscUtils::plot_point_sets( const cv::Mat& im, const cv::Mat& pts_set, cv::Mat& dst, const cv::Scalar& color, bool enable_keypoint_annotation, const string& msg )
{
  // TODO consider addressof(a) == addressof(b)
  // dst = im.clone();


  if( im.data == dst.data ) {
    //   cout << "src and dst are same\n";
      // src and dst images are same, so dont copy. just ensure it is a 3 channel image.
      assert( im.rows > 0 && im.cols > 0 && "\n[MiscUtils::plot_point_sets]Image appears to be emoty. cannot plot.\n");
      assert( im.channels() == 3 && dst.channels() == 3 && "[MiscUtils::plot_point_sets]src and dst image are same physical image in memory. They need to be 3 channel." );
  }
  else {
    //   dst = cv::Mat( im.rows, im.cols, CV_8UC3 );
      if( im.channels() == 1 )
        cv::cvtColor( im, dst, cv::COLOR_GRAY2BGR );
      else
        im.copyTo(dst);
  }

  // cv::putText( dst, to_string(msg.length()), cv::Point(5,5), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  if( msg.length() > 0 ) {
    vector<std::string> msg_split;
    msg_split = MiscUtils::split( msg, ';' );
    for( int q=0 ; q<msg_split.size() ; q++ )
      cv::putText( dst, msg_split[q], cv::Point(5,20+20*q), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  }


  //pts_set is 2xN
  cv::Point2d pt;
  for( int i=0 ; i<pts_set.cols ; i++ )
  {
    pt = cv::Point2d(pts_set.at<float>(0,i),pts_set.at<float>(1,i) );
    cv::circle( dst, pt, 2, color, -1 );

    char to_s[20];
    sprintf( to_s, "%d", i);
    if( enable_keypoint_annotation ) {
        // cv::putText( dst, to_s, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,255,255) - color  );
        cv::putText( dst, to_s, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, color  );
    }

  }
}


void MiscUtils::plot_point_sets( cv::Mat& im, const MatrixXd& pts_set,
                                        const cv::Scalar& color, const VectorXi& annotations, const string& msg )
{
    plot_point_sets( im, pts_set, im, color, annotations, msg );
}


void MiscUtils::plot_point_sets( cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                        const cv::Scalar& color, const VectorXi& annotations, const string& msg )

{
  // TODO consider addressof(a) == addressof(b)
  // dst = im.clone();


  assert( im.rows > 0 && im.cols > 0 && "\n[MiscUtils::plot_point_sets]Image appears to be emoty. cannot plot.\n");
  assert( pts_set.cols() > 0 && pts_set.cols() == annotations.rows() && "[MiscUtils::plot_point_sets] VectorXi annotation size must be equal to number of points. If you wish to use the default annotation ie. 0,1,...n use `true` instead. If you do not want annotation use `false`." );
  if( im.data == dst.data ) {
    //   cout << "src and dst are same\n";
      // src and dst images are same, so dont copy. just ensure it is a 3 channel image.
      assert( im.channels() == 3 && dst.channels() == 3 && "[MiscUtils::plot_point_sets]src and dst image are same physical image in memory. They need to be 3 channel." );
  }
  else {
    //   dst = cv::Mat( im.rows, im.cols, CV_8UC3 );
      if( im.channels() == 1 )
        cv::cvtColor( im, dst, cv::COLOR_GRAY2BGR );
      else
        im.copyTo(dst);
  }

  // cv::putText( dst, to_string(msg.length()), cv::Point(5,5), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  if( msg.length() > 0 ) {
    vector<std::string> msg_split;
    msg_split = MiscUtils::split( msg, ';' );
    for( int q=0 ; q<msg_split.size() ; q++ )
      cv::putText( dst, msg_split[q], cv::Point(5,20+20*q), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  }


  //pts_set is 2xN
  cv::Point2d pt;
  for( int i=0 ; i<pts_set.cols() ; i++ )
  {
    // pt = cv::Point2d(pts_set.at<float>(0,i),pts_set.at<float>(1,i) );
    pt = cv::Point2d( (float)pts_set(0,i), (float)pts_set(1,i) );
    cv::circle( dst, pt, 2, color, -1 );

    char to_s[20];
    sprintf( to_s, "%d", annotations(i) );
    cv::putText( dst, to_s, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, color  );


  }
}
