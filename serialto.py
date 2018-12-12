# native python stuff
from __future__ import print_function
from time import time

import numpy as np
import cv2
import os, traceback, sys, argparse, json, io
import serial, threading, Queue

parser = argparse.ArgumentParser()
parser.add_argument("port", type=str, help="The serial port.")
parser.add_argument("-b", "--baud", type=int, help="The baud rate.", default=115200)
args = parser.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=1)
poseQueue = Queue.Queue()

sampling_rate = 30
class worker:
    def __init__(self, queue, ser):
        self.queue = queue
        self.ser = ser

    def read_serial(self, ser):
        while True:
            rx = ser.readline()
            # byte = inp.encode('hex')
            self.queue.put(rx)
            # time.sleep(.1) FIXME, not necessary

    def lastPose(self):


    def go(self):
        th1 = threading.Thread(target=self.read_serial, args=[ser])
        th1.start()

class worker2:
    def __init__(self, queue, ser):
        self.queue = queue
        self.ser = ser

    def getrx(self,whereUat):
        while True:
            rx = self.queue.get()
            print (str(rx))

    def go(self):
        th2 = threading.Thread(target=self.getrx, args=[self.queue])
        th2.start()

try:
    #cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
    t1 = worker(poseQueue,ser)
    t1.go()
    t2 = worker2(poseQueue)
    t2.go2()
    time.sleep(10)
    t2._stop()
    t1._stop()

finally:
    print("done")
