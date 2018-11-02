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
