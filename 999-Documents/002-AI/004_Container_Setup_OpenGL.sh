@@@ Container setup (with OpenGL)
@@@ VincentChan
@@@ 14th-March 2020

**Pay attention to the docker mapping path
**Pay attnetion to the mapping port
**This configuration supports OpenGL
**Pay atthention to the mapping video device (camera is webcam)

$ xhost +
$ sudo docker run --gpus all -it --device=/dev/video0:/dev/video0 -v /home/vincent/docker:/workspace/ -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all --env QT_X11_NO_MITSHM=1 -p 2002:8888 nvcr.io/nvidia/tensorflow:19.07-py3
