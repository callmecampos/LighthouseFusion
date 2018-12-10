# native python stuff
from __future__ import print_function
from time import time
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt


# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import os, traceback, sys, argparse, json, io

import serial 
ser = serial.Serial('COM4',9600, timeout=1) 
pose_queue = list()
pose_queue_floats =list()

sampling_rate= 30;
try:
	cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
	i=0
	j=0
	while True:
		pose_queue.insert(i,ser.readline())
		# print (pose_queue[i])
		
	#	while j<10:

		pose_queue_floats= map(float, pose_queue[i].split(','))
		print(pose_queue_floats[0])
		print(pose_queue_floats[1])
		print(pose_queue_floats[2])
		key = cv2.waitKey(1)
		# Press esc or 'q' to close the image window
		if key & 0xFF == ord('q') or key == 27:
			break
		i+=1	
	# for pose in pose_queue:
		# print(pose)
finally:
    print("done")