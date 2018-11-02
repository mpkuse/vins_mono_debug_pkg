/** Testing Opening files
**/

#include <iostream>

#include "utils/RawFileIO.h"
#include "utils/PoseManipUtils.h"

const string base_path = "/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/";
int main()
{
    string fname;

    //
    // Read K matrix
    fname = base_path+"/cameraIntrinsic.K";
    Matrix3d K;
    RawFileIO::read_eigen_matrix( fname, K );
    cout << "K\n" << K << endl;

    //
    // Read D (distortion)
    fname = base_path+"/cameraIntrinsic.D";
    MatrixXd D;
    RawFileIO::read_eigen_matrix( fname, D );
    cout << "D\n" << D << endl;

    //
    // Read Pose (as a 4x4 matrix)
    fname = base_path+"/3741.wTc";
    Matrix4d wTc;
    RawFileIO::read_eigen_matrix( fname, wTc );
    cout << "wTc\n" << wTc << endl;
    cout << "wTc(Pretty print)\n" << PoseManipUtils::prettyprintMatrix4d( wTc ) << endl;


    fname = base_path+"/3444.cX.pointcloud";
    MatrixXd cX;
    RawFileIO::read_eigen_matrix( fname, cX );
    cout << "cX.shape = " << cX.rows() << "x" << cX.cols() << endl;
    cout << "cX\n" << cX << endl;

    fname = base_path+"/3444.uv";
    MatrixXd uv;
    RawFileIO::read_eigen_matrix( fname, uv );
    cout << "uv.shape = " << uv.rows() << "x" << uv.cols() << endl;
    cout << "uv\n" << uv << endl;



    return 0;

}
