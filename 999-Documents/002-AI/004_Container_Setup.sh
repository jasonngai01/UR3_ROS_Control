@@@ Container setup (without OpenGL)
@@@ VincentChan
@@@ 14th-March 2020

**Pay attention to the docker mapping path
**Pay attnetion to the mapping port
**This configuration does not support OpenGL


$ xhost +
$ sudo docker run --gpus all -it -v /home/vincent/Desktop/Docker/Research:/Docker/Research -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 -p 2000:8888 nvcr.io/nvidia/tensorflow:19.07-py3
