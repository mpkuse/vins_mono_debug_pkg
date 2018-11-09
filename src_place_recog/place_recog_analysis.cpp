/** Loads the files from _tmp_cerebro folder. Main purpose of this is to
    test performance of DBOW2 and ibow_lcd.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 9th Nov, 2018
**/

#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
using namespace std;


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "DBoW3.h"
using namespace DBoW3;


#include "../utils/PoseManipUtils.h"
#include "../utils/RawFileIO.h"

#include "../utils/nlohmann/json.hpp"
using json = nlohmann::json;


// const std::string BASE = "/Bulk_Data/_tmp_cerebro/bb4_long_lab_traj/";
const std::string BASE = "/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/";


int main_jsontest()
{
    // create a JSON object
    json j =
    {
        {"pi", 3.141},
        {"happy", true},
        {"name", "Niels"},
        {"nothing", nullptr},
        {
            "answer", {
                {"everything", 42}
            }
        },
        {"list", {1, 0, 2}},
        {
            "object", {
                {"currency", "USD"},
                {"value", 42.99}
            }
        }
    };

    // add new values
    j["new"]["key"]["value"] = {"another", "list"};

    // count elements
    auto s = j.size();
    j["size"] = s;

    // pretty print with indent of 4 spaces
    std::cout << std::setw(4) << j << '\n';
}


int main( int argc, char ** argv )
{
    //
    // Load Log file (.txt)
    //

    //
    // Load json
    //
    string json_fname = BASE+"/log.json";
    cout << "Open file: " << json_fname << endl;
    std::ifstream json_fileptr(json_fname);
    json j;
    json_fileptr >> j;
    cout << "Successfully opened file "<< json_fname << endl;
    // std::cout << std::setw(4) << j << '\n';

    // Collect all poses
    std::cout << "j[\"DataNodes\"].size()  " << j["DataNodes"].size() << std::endl;
    vector< string > list_of_imfiles;
    vector< Matrix4d > list_of_vio_poses;
    for( int i=0 ; i<200 ; i++ ) {
        if(  j["DataNodes"][i]["isPoseAvailable"] == 1 ) {
            cout << i << " isPoseAvailable: " <<  j["DataNodes"][i]["isPoseAvailable"] << endl;
            // Matrix4d w_T_c_from_file;
            // RawFileIO::read_eigen_matrix( BASE+"/"+to_string(i)+".wTc", w_T_c_from_file );
            // cout << "w_T_c_from_file "<< w_T_c_from_file << endl;


            Matrix4d w_T_c;
            vector<double> raa = j["DataNodes"][i]["w_T_c"]["data"];
            RawFileIO::read_eigen_matrix( raa, w_T_c );
            cout << "w_T_c "<< w_T_c << endl;

            string im_fname = BASE+"/"+to_string(i)+".jpg";
            list_of_imfiles.push_back( im_fname );
            list_of_vio_poses.push_back( w_T_c );
        }
    }



    //
    //  Init DBOW
    //
    Vocabulary voc("../orbvoc.dbow3");



}
