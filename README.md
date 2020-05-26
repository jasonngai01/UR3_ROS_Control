# UR3_ROS_Control

Acupuncture Positioning System
-------------------------------------------
This repository is an acupuncture positioning system for traditional chinese medicine. With the help of Aritifical Intelligence, several acupuncutre points are recognized using **Nvidia Jetson TX2**. Meanwhile, **UR3** is responsible to execute the acupuncture therapy.

### Acupuncture Point Recognition
Several acupuncture points on the forearm of a person can be recognized. In the current stage, the focus will be acupuncture points with index 0,1,2,3 and 4. The details of those indices are as following:
* Index 0: *PC07-Daling*
* Index 1: *HT07-Shenmen*
* Index 2: *LU09-Taiyuan*
* Index 3: *LU05-Chize*
* Index 4: *HT03-Shaohai*

![image](https://github.com/vincent51689453/UR3_ROS_Control/blob/master/Git_Image/detector.png)

This is the detection output of Nvidia Jetson TX2 and Tensorflow 1.14.0.

### Simulation of acupuncture therapy
UR3 transforms those coordiantes of different acupuncture points into a its own coordinate system. Afterward, it will approach to those points for therapy.

![image](https://github.com/vincent51689453/UR3_ROS_Control/blob/master/Git_Image/ACP_Version_2_Output.gif)
