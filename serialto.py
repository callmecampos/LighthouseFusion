# native python stuff
from __future__ import print_function
from time import time

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import os, traceback, sys, argparse, json, io

import serial 
ser = serial.Serial('/dev/ttyACM0',500000, timeout=1) 
pose_queue = list()
try:
    i=0
    while i<10:
    
        pose_queue.insert(i,ser.readline())
        i+=1
    i=0
    while i<10:
    
        print(pose_queue[i])   
        i+=1
finally:
    print("done")
