# native python stuff
from __future__ import print_function
from time import time


# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
#import cv2
import os, traceback, sys, argparse, json, io

import serial 
ser = serial.Serial('COM4',9600, timeout=1) 
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
