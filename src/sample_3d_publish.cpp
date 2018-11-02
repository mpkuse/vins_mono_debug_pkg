/** Read a 3d pointcloud and publish it **/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <sstream>


#include <iostream>

#include "utils/RawFileIO.h"
#include "utils/PoseManipUtils.h"
#include "utils/RosMarkerUtils.h"

const string base_path = "/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_3d_pts" );
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("chatter", 1000);


    string fname;
    //
    // Read Pose (as a 4x4 matrix)
    fname = base_path+"/533.wTc";
    Matrix4d wTc;
    RawFileIO::read_eigen_matrix( fname, wTc );
    cout << "wTc\n" << wTc << endl;
    cout << "wTc(Pretty print)\n" << PoseManipUtils::prettyprintMatrix4d( wTc ) << endl;


    //
    // Make Camera Marker
    visualization_msgs::Marker cam;
    RosMarkerUtils::init_camera_marker( cam, 1.0 );
    cam.ns = "camera";
    cam.id = 0;
    RosMarkerUtils::setpose_to_marker( wTc, cam );


    //
    // Read point cloud
    fname = base_path+"/533.wX.pointcloud";
    MatrixXd wX;
    RawFileIO::read_eigen_matrix( fname, wX );

    //
    // Make Point Cloud Marker
    visualization_msgs::Marker ptcld_marker;
    RosMarkerUtils::init_points_marker( ptcld_marker );
    ptcld_marker.ns = "wX";
    ptcld_marker.id = 0;
    ptcld_marker.scale.x = 0.04;
    ptcld_marker.scale.y = 0.04;
    RosMarkerUtils::add_points_to_marker( wX, ptcld_marker );







    ros::Rate loop_rate(10);
    int count =0;
    while (ros::ok())
    {
        chatter_pub.publish( cam );
        chatter_pub.publish( ptcld_marker );

        for( int i=0 ; i<wX.cols() ; i++ ) // text at each point
        {
            visualization_msgs::Marker _txt;
            RosMarkerUtils::init_text_marker( _txt );
            _txt.ns = "wX_txt";
            _txt.id = i;
            _txt.text = to_string(i);
            _txt.scale.z = 0.1;
            // Vector4d wX_i =
            RosMarkerUtils::setposition_to_marker( (Vector4d) wX.col(i) , _txt );
            chatter_pub.publish( _txt );
        }
        ros::spinOnce();
        loop_rate.sleep() ;
    }
}
