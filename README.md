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
Uses `CMakeLists.standalone.txt`. Mainly has util for intrinsic camera calib for
monocular and stereo camera.

```
mkdir build
cmake .. -DSTANDALONE=ON
make
```

calibration. About 50-100 images are enought.
```
./intrinsic_calib -i /Bulk_Data/ros_bags/mynteye/calib/calib1/ -p cam0_ -e .png -w 8 -h 12 -s 80

./stereo_calib -i /Bulk_Data/ros_bags/mynteye/calib/calib1_sampled/  --prefix-l cam0_ --prefix-r cam1_ -e .png -w 8 -h 12 -s 80
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

### TheiaSFM Installation
In `src_geometry`, I am trying some algorithms from [TheiaSFM](http://www.theia-sfm.org/).

- Install Open Image IO. [github-repo](https://github.com/OpenImageIO/oiio)
    ```
    $ git checkout Release-1.7.6RC1
    $ make -j 12
    $ cd dist/linux64
    $ #since there is no make install, will have to manually copy include, lib, share
    ```
- Install rocksdb
    ``` $ apt-get install libsnappy-dev # snappy
        $ make shared_lib
        $ make install
    ```
- Install TheiaSFM
