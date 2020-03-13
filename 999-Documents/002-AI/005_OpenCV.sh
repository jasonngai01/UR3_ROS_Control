@@@ Install Opencv 3.4.0 in docker container
@@@ VincentChan
@@@ 14th-March 2020

1) Install following dependencies
   $ apt-get update

   $ apt-get upgrade

   $ apt-get install build-essential cmake pkg-config

   $ apt-get install software-properties-common

   $ add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"

   $ apt update

   $ apt-get install libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev

   $ apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

   $ apt-get install libxvidcore-dev libx264-dev

   $ apt-get install libgtk-3-dev

   $ apt-get install libatlas-base-dev gfortran


2) Go to "/" directory
   $ wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.4.0.zip

   $ unzip opencv.zip

   $ wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.4.0.zip

   $ unzip opencv_contrib.zip

3) Go to "/opencv-3.4.0"
   $ mkdir build
 
   $ cd build

   $ cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib-3.4.0/modules \
    -D PYTHON_EXECUTABLE=/usr/bin/python \
    -D BUILD_EXAMPLES=ON ..

4) Check the installation report, especially for "Python3" session. Also, you should be able to see "Configuration Done" and "Generation Done"

5) Make opencv
   $ make -j8 where the number of j depends on the number of cpu cores


6) Make install
   $ make install
   $ ldconfig

Reference: https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/
