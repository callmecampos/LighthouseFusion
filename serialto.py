# native python stuff
from __future__ import print_function
from time import time


# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import os, traceback, sys, argparse, json, io
import threading
import Queue
import time

import serial 
ser = serial.Serial('COM3',9600, timeout=1)
whereUat = Queue.Queue()


sampling_rate= 30
class worker: 
    def __init__(self,whereUat,ser): 
        self.whereUat = whereUat 
        self.ser = ser 

    def read_serial(self,ser): 
        while True : 
            rx=ser.readline()
            #byte = inp.encode('hex')
            self.whereUat.put(rx)
            time.sleep(.1)

    def go(self): 
        th1 = threading.Thread(target=self.read_serial, args=[ser]) 
        th1.start() 

class worker2: 
    def __init__(self,whereUat,): 
        self.whereUat = whereUat 
        self.ser = ser 

    def getrx(self,whereUat): 
        while True:
            rx = self.whereUat.get()
            print (str(rx))

    def go2(self): 
        th2 = threading.Thread(target=self.getrx, args=[whereUat])
        th2.start() 

try:
    #cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
    t = worker(whereUat,ser)
    t.go()
    t2 = worker2(whereUat) 
    t2.go2()
    time.sleep(10)
    t2._stop()
    t1._stop()
        # if key & 0xFF == ord('q') or key == 27:
            
            # break
finally:
    print("done")