from __future__ import print_function
from multiprocessing import Process, Queue
import os, traceback, sys, argparse, io, time
import serial, multiprocessing

curr_pose = []

class SerialWorker(Process):
    def __init__(self, port, baud=115200, timeout=1):
        super(SerialWorker, self).__init__(target=self.loop_iterator, args=(port, baud, timeout))

    def loop_iterator(self, port, baud, timeout):
        global curr_pose
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        for new_line in self.loop(ser):
            pose = [time.time()]
            data = new_line.split("\t")
            pose.extend(data)
            if data[0] == "Setup:" or data[0] == "Error:":
                continue

            print(pose)
            curr_pose = pose

    def loop(self, ser):
        while True:
            yield ser.readline().decode('utf-8')

        # ser.close # may not be necessary?

def main():
    worker = SerialWorker(args.port, args.baud)
    worker.start()

    for i in range(20):
        print(curr_pose)
        time.sleep(2)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", type=str, help="The serial port.")
    parser.add_argument("-b", "--baud", type=int, help="The baud rate.", default=115200)
    args = parser.parse_args()

    main()
