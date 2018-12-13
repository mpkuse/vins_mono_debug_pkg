Code adopted from https://github.com/rpng/calc/tree/master/TrainAndTest.

Follow Nate Merill's (https://github.com/rpng/calc) instruction
to download the pretrained caffemodel and put them in folder `proto`


# How to run

I used caffe docker to execute this:
```
nvidia-docker run -ti -v /home/mpkuse/docker_ws_slam/:/app -v /media/mpkuse/Bulk_Data/:/Bulk_Data bvlc/caffe:gpu  bash
pip install opencv-python
```
