
void publish_vio_poses( ros::Publisher& pub, vector<Matrix4d>& list_of_vio_poses, bool line_strip=true, bool cam_viz=false, bool text=false )
{
    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_strip_marker( marker );
    marker.ns = "vio_poses";
    marker.id = 0;
    RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.0, marker );

    visualization_msgs::Marker marker_cam;
    RosMarkerUtils::init_camera_marker( marker_cam, 1.0 );
    marker_cam.ns = "vio_poses_camera";


    visualization_msgs::Marker marker_text;
    RosMarkerUtils::init_text_marker( marker_text );
    marker_text.ns = "vio_poses_text";
    marker_text.scale.z = 0.07;
    RosMarkerUtils::setcolor_to_marker( 1.0, 1.0, 1.0, marker_text );
    for( int i=0 ; i<list_of_vio_poses.size() ; i++ )
    {
        if( line_strip ) {
            geometry_msgs::Point w_t_c;
            w_t_c.x = list_of_vio_poses[i](0,3);
            w_t_c.y = list_of_vio_poses[i](1,3);
            w_t_c.z = list_of_vio_poses[i](2,3);
            marker.points.push_back( w_t_c );
        }

        if( cam_viz ) {
            RosMarkerUtils::setpose_to_marker( list_of_vio_poses[i] , marker_cam );
            marker_cam.id = i;
            pub.publish( marker_cam );
        }

        if( text ) {
            RosMarkerUtils::setpose_to_marker( list_of_vio_poses[i] , marker_text );
            marker_text.text = to_string( i );
            marker_text.id = i;
            pub.publish( marker_text );
        }
    }
    pub.publish( marker );
}

void publish_loop_edges( ros::Publisher& pub,
                         vector<Matrix4d>& list_of_vio_poses,
                         vector< tuple<int,int,double> >& list_of_scores
                      )
{
    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_marker( marker );
    marker.ns = "loop_edges";
    marker.id = 0;
    RosMarkerUtils::setcolor_to_marker( 1.0, 0.0, 0.0, marker );


    // iterate over all the candidates
    for( int i=0 ; i<list_of_scores.size() ; i++ ) {
        int curr_i = std::get<0>(list_of_scores[i]);
        int prev_i = std::get<1>(list_of_scores[i]);
        double score = std::get<2>(list_of_scores[i]);

        Vector4d w_t_curr = list_of_vio_poses[curr_i].col(3);
        Vector4d w_t_prev = list_of_vio_poses[prev_i].col(3);
        RosMarkerUtils::add_point_to_marker( w_t_curr, marker, false );
        RosMarkerUtils::add_point_to_marker( w_t_prev, marker, false );
    }
    pub.publish( marker );
}

void imshow_loopcandidates( vector<string> list_of_imfiles, vector< tuple<int,int,double> >& list_of_scores )
{
    for( int i=0 ; i<list_of_scores.size() ; i++ ) {
        int curr_i = std::get<0>(list_of_scores[i]);
        int prev_i = std::get<1>(list_of_scores[i]);
        double score = std::get<2>(list_of_scores[i]);

        std::stringstream ss;
        ss << "[" << i << " of " << list_of_scores.size() << "] ";
        ss << "curr_i="<< curr_i << "    ";
        ss << "prev_i="<< prev_i << "    ";
        ss << "Score="<< score ;
        cout << ss.str() << endl;

        cv::Mat im_curr_i = cv::imread( list_of_imfiles[curr_i]);
        cv::Mat im_prev_i = cv::imread( list_of_imfiles[prev_i]);

        cv::Mat dsy;
        cv::hconcat( im_curr_i, im_prev_i , dsy );

        cv::Mat dsy_status = cv::Mat::zeros( 100, dsy.cols, dsy.type() );
        cv::putText(dsy_status, ss.str().c_str(), cvPoint(10,35), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);


        cv::Mat to_show;
        cv::vconcat( dsy,dsy_status,to_show );

        cv::imshow( "loopcandidate", to_show );
        // cv::imshow( "im_curr_i", im_curr_i );
        // cv::imshow( "im_prev_i", im_prev_i );
        char key = cv::waitKey(0);
        if( key == 'q' )
            break;
    }

    // cv::destroyWindow( "im_curr_i" );
    // cv::destroyWindow( "im_prev_i" );
    cv::destroyWindow( "loopcandidate" );

}
