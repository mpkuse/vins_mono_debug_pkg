/** Loads the files from _tmp_cerebro folder. Main purpose of this is to
    test performance of DBOW2 and ibow_lcd.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 9th Nov, 2018
**/

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>
using namespace std;


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "DBoW3.h"
using namespace DBoW3;


#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>



#include "../utils/PoseManipUtils.h"
#include "../utils/RawFileIO.h"
#include "../utils/RosMarkerUtils.h"
#include "../utils/TermColor.h"

#include "unit_tools.h" // publish utils for this file.


#include "../utils/nlohmann/json.hpp"
using json = nlohmann::json;


// const std::string BASE = "/Bulk_Data/_tmp_cerebro/bb4_long_lab_traj/";
// const std::string BASE = "/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/";
// const std::string BASE = "/Bulk_Data/_tmp_cerebro/bb4_loopy_drone_fly_area/";

const std::string BASE = "/Bulk_Data/_tmp_cerebro/mynt_drone_fly_area_loopy/";


// #define __place_recog_dbow3_cpp_loadFeatures( msg ) msg
#define __place_recog_dbow3_cpp_loadFeatures( msg )
// Given a list of image files, will compute the ORB descriptors for each of the images.
vector< cv::Mat  >  loadFeatures( std::vector<string> path_to_images,string descriptor="orb") throw (std::exception){
    //select detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptor=="orb")        fdetector=cv::ORB::create();
    else if (descriptor=="brisk") fdetector=cv::BRISK::create();
    else throw std::runtime_error("Invalid descriptor");
    assert(!descriptor.empty());
    vector<cv::Mat>    features;


    cout << "Extracting   features...for "<< path_to_images.size() << " images" << endl;
    cout << "[0] "<< path_to_images[0] << endl;
    cout << "[" << path_to_images.size()-1 << "]" << path_to_images[ path_to_images.size()-1 ] << endl;
    for(size_t i = 0; i < path_to_images.size(); ++i)
    {
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        __place_recog_dbow3_cpp_loadFeatures(cout<<"reading image: "<<path_to_images[i]<<endl;)
        cv::Mat image = cv::imread(path_to_images[i], 0);
        if(image.empty())throw std::runtime_error("Could not open image"+path_to_images[i]);
        __place_recog_dbow3_cpp_loadFeatures(cout<<"extracting features"<<endl;)
        fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
        features.push_back(descriptors);
        __place_recog_dbow3_cpp_loadFeatures(cout << "# of keypoints : "<< keypoints.size() << "\t";)
        __place_recog_dbow3_cpp_loadFeatures(cout << "descriptors shape : "<< descriptors.rows << "x" << descriptors.cols << "\t";)
        __place_recog_dbow3_cpp_loadFeatures(cout<<"done detecting features"<<endl;)
    }
    return features;
}




void filter_candidates( const vector< std::tuple< int, int, double > >& src,
                        vector< std::tuple< int, int, double > >& dst,
                        double TH, int locality, bool enable_print )
{
    // TODO: Geometric verification. using F-test
    dst.clear();

    // iterate over all the candidates and get rid of candidates which don't follow locality rule.
    cout << "src.size=" << src.size() << endl;
    for( int i=0 ; i<src.size() ; i+=3 ) {
        int curr_i = std::get<0>(src[i]);
        int prev_i = std::get<1>(src[i]);
        double score = std::get<2>(src[i]);

        if( score > 0.01 && score < TH && abs(std::get<1>(src[i+1])-prev_i)< locality && abs(std::get<1>(src[i+2])-prev_i) < locality )
        {
            if( enable_print )
                cout << curr_i  << "< " << score << " >" << prev_i << endl;

            dst.push_back( std::make_tuple( curr_i, prev_i, score ) );
        }
    }
}

int main( int argc, char ** argv )
{
    //
    // ROS init
    std::string cur_rospkg_path = ros::package::getPath("vins_mono_debug_pkg");
    ros::init(argc, argv, "place_recog_dbow3" );
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("chatter", 1000);
    // TODO pub_frames, pub_loopcandidates.




    //
    // Load json
    //
    string json_fname = BASE+"/log.json";
    cout << TermColor::GREEN() << "Open file: " << json_fname << TermColor::RESET() <<  endl;
    std::ifstream json_fileptr(json_fname);
    json json_obj;
    json_fileptr >> json_obj;
    cout << "Successfully opened file "<< json_fname << endl;
    // std::cout << std::setw(4) << j << '\n';

    // Collect all poses
    std::cout << "json_obj[\"DataNodes\"].size()  " << json_obj["DataNodes"].size() << std::endl;
    vector< string > list_of_imfiles;
    vector< Matrix4d > list_of_vio_poses;
    vector< int >  list_of_idx;
    // int n_max = 1500;
    int n_max = json_obj["DataNodes"].size();
    for( int i=0 ; i<n_max ; i++ ) {
        if(  json_obj["DataNodes"][i]["isPoseAvailable"] == 1 ) {
            cout << i << " isPoseAvailable: " <<  json_obj["DataNodes"][i]["isPoseAvailable"] << endl;
            // Matrix4d w_T_c_from_file;
            // RawFileIO::read_eigen_matrix( BASE+"/"+to_string(i)+".wTc", w_T_c_from_file );
            // cout << "w_T_c_from_file "<< w_T_c_from_file << endl;


            Matrix4d w_T_c;
            vector<double> raa = json_obj["DataNodes"][i]["w_T_c"]["data"];
            RawFileIO::read_eigen_matrix( raa, w_T_c );//loads as row-major
            cout << "w_T_c "<< w_T_c << endl;

            string im_fname = BASE+"/"+to_string(i)+".jpg";
            list_of_imfiles.push_back( im_fname );
            list_of_vio_poses.push_back( w_T_c );
            list_of_idx.push_back( i );
        }
    }


    //
    //  Init DBOW
    //
    // cout << "Loading the pretrained vocabulary. If this file was not found there is a script in `" << cur_rospkg_path << "/utils/dbow3_core/get_orbvoc.dbow3" << "` to download the vocabulary\n";
    // Init Vocabulary
    string fname_vocab_dbow3 = cur_rospkg_path+"/utils/dbow3_core/orbvoc.dbow3";
    cout << TermColor::GREEN() << "Load DBOW3 Vocab file : " << fname_vocab_dbow3 << TermColor::RESET() << endl;
    Vocabulary voc( fname_vocab_dbow3 );
    // Init a database
    Database db(voc, false, 0);

    //
    // Extract ORB features for each image
    //
    vector< cv::Mat  > all_desc = loadFeatures( list_of_imfiles );
    cout << "all_desc.size() = " << all_desc.size() << endl;


    //
    // add images to the database
    //
    vector< std::tuple< int, int, double > > loop_candidates_list;
    for(size_t i = 0; i < all_desc.size(); i++)
    {
        QueryResults ret;
        db.query(all_desc[i], ret, 20 );
        cout << "Searching for Image " << i << ". "  << ret << endl;

        if( ret.size() == 0 ) {
            loop_candidates_list.push_back( std::make_tuple( i, -1, 0.0 ) );
        }
        else {
            if( ret.size() == 1 )
                loop_candidates_list.push_back( std::make_tuple( i, ret[0].Id, 1.0 ) );
            else if( ret.size() > 1 )
                loop_candidates_list.push_back( std::make_tuple( i, ret[0].Id, ret[1].Score/ret[0].Score ) );
        }

        BowVector bow_vec;
        int indx_to_add = i-200;
        if( indx_to_add > 0 ) {
            cout << "Add image #"<< indx_to_add << "'s features to db\n";
            db.add(all_desc[indx_to_add], &bow_vec );
            // cout << "bow_vec.size() : "<< bow_vec.size() << endl;
        }

    }

    cout << "Database information: " << endl << db << endl;

    vector< std::tuple< int, int, double > > loop_candidates_list_filtered;
    // TODO : This is a locality and threshold filter. Do geometric filtering.
    filter_candidates( loop_candidates_list, loop_candidates_list_filtered, 1.0, 8, true );


#if 1
    // Save to json file:
    json jsonout_obj;
    cout << "Save to JSON : #candidates : " << loop_candidates_list_filtered.size() << endl;
    for( int i=0; i<loop_candidates_list_filtered.size() ; i++ )
    {
        int curr_i = std::get<0>(loop_candidates_list_filtered[i]);
        int prev_i = std::get<1>(loop_candidates_list_filtered[i]);
        double score = std::get<2>(loop_candidates_list_filtered[i]);

        int global_curr_i = 0;
        int global_prev_i = 0;

        json _cur_obk;
        _cur_obk["a"] = (int) curr_i;
        _cur_obk["b"] = (int) prev_i;
        _cur_obk["inliers"] = score;
        // _cur_obk["score"] = score;
        _cur_obk["fname_a"] = list_of_imfiles[curr_i];
        _cur_obk["fname_b"] = list_of_imfiles[prev_i];
        _cur_obk["global_a"] = list_of_idx[curr_i];
        _cur_obk["global_b"] = list_of_idx[prev_i];
        jsonout_obj.push_back( _cur_obk );
    }

    // write json object to file
    std::string json_out_fname_org = BASE+"/loopcandidates_dbow.json";
    std::cout << "Write File: " << json_out_fname_org << endl;
    std::ofstream out( json_out_fname_org );
    out << jsonout_obj.dump(4);
    out.close();
    return 0;
#endif


    // publish
    ros::Rate loop_rate(10);
    cv::Mat status_image = cv::Mat::zeros(120,450, CV_8UC3);
    double TH = 0.7;
    while( ros::ok() )
    {
        cout << TermColor::RED() << "---" << TermColor::RESET() << TH << endl;
        filter_candidates( loop_candidates_list, loop_candidates_list_filtered, TH, 8, false );


        // Make status image
        status_image = cv::Mat::zeros(120,450, CV_8UC3);
        cv::putText(status_image, BASE, cvPoint(10,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);
        string _msg = "Accepted="+to_string(loop_candidates_list_filtered.size())+ " / "+to_string( loop_candidates_list.size() );
        cv::putText(status_image, _msg.c_str(), cvPoint(10,35), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);
        cv::putText(status_image, to_string(TH), cvPoint(10,65), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);



        publish_vio_poses( pub, list_of_vio_poses, true, true, true );
        publish_loop_edges( pub, list_of_vio_poses, loop_candidates_list_filtered );

        cv::imshow( "status_image", status_image );
        char key = cv::waitKey(10);
        if( key == 'a' )
            TH+=0.01;
        if( key == 'z' )
            TH-=0.01;
        if( key == 'q' )
            break;
        if( key == 's' )
            imshow_loopcandidates( list_of_imfiles, loop_candidates_list_filtered );


        loop_rate.sleep();
    }
}
