#!/usr/bin/env python
import sys
import urx
import time
#import file
import RG2Gripper as GripClass
rob = urx.Robot("158.132.153.174")
#instantiate object
gripperInstance = GripClass.RG2(rob)

#basic delay for script(to complete action, RG2 approximatly takes 1.3 seconds to close from completely open)
time.sleep(2)
#set distance between pincers apart, in this case closing
gripperInstance.setWidth(0,force =30)
time.sleep(2)
#returns the current width of the robot
gripperInstance.getWidth()
#end of process
rob.close()

