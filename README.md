# Debug ROS Package for VINS-mono.

This is designed to read the dump file and do custom analysis. In the dump folder you should see:
- log.json
- log.txt
- cameraIntrinsic.K
- cameraIntrinsic.D
- cameraIntrinsic.info
- for all i in range(0,n)
    - **i.jpg** #the image
    - **i.id**  #global ids of the tracked features Nx1
    - **i.unvn** #tracked points in normalized image cords
    - **i.uv**   #tracked points raw
    - **i.cX.pointcloud** # 3d points estimated from raws tracks. cX.
    - **i.wX.pointcloud** # 3d points estimated from raws tracks. wX. world here is the 1st frame of the odometry. It ofcourse drifts as time passes.
    - **i.wTc**

I am happy to provide a sample dump folder as a tarball. Email if you want to try.

### StandAlone Make
Uses `CMakeLists.standalone.txt`

```
mkdir build
cmake .. -DSTANDALONE=ON
make
```

### Catkin Compilation
Uses `CMakeLists.rospkg.txt`

This can also be put in a catkin_ws and will compile with catkin_make.


### Directory Structure
- src_geometry : Scripts (both py and cpp) for experiments related to geometry and pose computation
- src_place_recog : Scripts related to experiements on recognizing places.

- include : ros generated include
- rviz : .rviz files
- utils : my collection of cpp utils. This can be included in cpp programs
