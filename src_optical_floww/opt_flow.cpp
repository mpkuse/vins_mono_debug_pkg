/*
    Go through logged file and compute optical flow
*/


#include <iostream>
#include <string>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/video/tracking.hpp>

#include "../utils/MiscUtils.h"
#include "../utils/ElapsedTime.h"
#include "../utils/PoseManipUtils.h"
#include "../utils/TermColor.h"
#include "../utils/RawFileIO.h"
#include "../utils/RosMarkerUtils.h"

#include "../utils/nlohmann/json.hpp"
using json = nlohmann::json;

const std::string BASE = "/Bulk_Data/_tmp/";


void loop_thru()
{
    ElapsedTime timer;

    // Open JSON log file
    string json_fname = BASE+"/log.json";
    cout << TermColor::GREEN() << "Open file: " << json_fname << TermColor::RESET() <<  endl;
    std::ifstream json_fileptr(json_fname);
    json json_obj;
    json_fileptr >> json_obj;
    cout << "Successfully opened file "<< json_fname << endl;


    cv::Mat prev_im;
    cv::Mat curr_im;

    cv::Mat curr_pts;
    MatrixXd curr_pts_e;

    cv::Mat prev_pts;
    MatrixXd prev_pts_e;
    for( int i=0 ; i<json_obj["DataNodes"].size() ; i++ )
    {
        cout << "==============" << json_obj["DataNodes"][i]["isImageAvailable"] << "  t=" << json_obj["DataNodes"][i]["getT"] << endl;

        curr_im = cv::imread( BASE+to_string(i)+".jpg", 0);
        cout << "curr_im : "<< MiscUtils::cvmat_info( curr_im ) << endl;

        // GoodFeaturesToTrack
        timer.tic();
        cv::goodFeaturesToTrack( curr_im, curr_pts, 50, 0.01, 45 );
        cout << "cv::goodFeaturesToTrack() in ms=" << timer.toc_milli() << endl;

        timer.tic();
        MiscUtils::cvmatpts_2_eigen( curr_pts, curr_pts_e );
        cout << "cv::MiscUtils::cvmatpts_2_eigen() in ms=" << timer.toc_milli() << endl;
        cout << "curr_pts : " << MiscUtils::cvmat_info( curr_pts ) << endl;

        #if 0
        cv::Mat dst;
        MiscUtils::plot_point_sets( curr_im, curr_pts_e, dst, cv::Scalar(0,0,255) );
        cv::imshow( "dst", dst );
        #endif

        if( !curr_im.empty() && !prev_im.empty() )
        {
            // cv::imshow( "curr_im", curr_im );
            // cv::imshow( "prev_im", prev_im );


            cv::Mat status, err, nextPts;
            // nextPts = prev_pts;
            timer.tic();
            calcOpticalFlowPyrLK( prev_im, curr_im, prev_pts, nextPts, status, err );
            cout << "calcOpticalFlowPyrLK done in ms=" << timer.toc_milli() << endl;
            // cout << "status = " << MiscUtils::cvmat_info( status ) << endl;
            // cout << "err = " << MiscUtils::cvmat_info( err ) << endl;
            cout << "nextPts = " << MiscUtils::cvmat_info( nextPts ) << endl;

            int m=0;
            for( int s=0 ; s<status.rows ; s++ ) {
                cout << s << " status=" <<  (int)status.at<uchar>(s) << " err=" << err.at<float>(s);
                cout << " prev_pt=" << prev_pts.at<cv::Vec2f>(s)[0] << "," << prev_pts.at<cv::Vec2f>(s)[1];
                cout << " nextPts=" << nextPts.at<cv::Vec2f>(s)[0] << "," << nextPts.at<cv::Vec2f>(s)[1] << endl;
                if( status.at<uchar>(s) > 0 ) m++;
            }
            cout << "m=" << m << endl;

            {
                MatrixXd nextPts_e;
                MiscUtils::cvmatpts_2_eigen( nextPts, nextPts_e );
                cv::Mat dst;
                MiscUtils::plot_point_sets( curr_im, nextPts_e, dst, cv::Scalar(255,0,0), true );
                cv::imshow( "nextPts_e on curr", dst );
            }

            cv::waitKey(0);


        }

        // book keeping
        if( i%10==0 ) {
            cout << "seeting kf\n";
        prev_im = curr_im;
        prev_pts = curr_pts;
        prev_pts_e = curr_pts_e;
        #if 1
        {
        cv::Mat dst;
        MiscUtils::plot_point_sets( prev_im, prev_pts_e, dst, cv::Scalar(0,0,255) );
        cv::imshow( "annotated KF", dst );
        }
        #endif

        }
    }

}

int main()
{
    loop_thru();
    cout << "\n...Done...!\n";
}
