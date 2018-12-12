from __future__ import print_function
from multiprocessing import Process, Queue
import os, traceback, sys, argparse, io, time
import serial, multiprocessing, Queue

class SerialWorker(Process):
    def __init__(self, port, baud=115200, timeout=1, queue=Queue()):
        self.queue = queue
        super(SerialWorker, self).__init__(target=self.loop_iterator, args=(port, baud, timeout, queue))

    def loop_iterator(self, port, baud, timeout, queue):
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        t = None
        for new_line in self.loop(ser, queue):
            t = time()
            data = line.split("\t")
            if data[0] == "Setup:" or data[0] == "Error:":
                continue

            queue.put(data)

    def loop(self, ser):
        while True:
            yield ser.readline().decode('utf-8')

        # ser.close # may not be necessary?

def main():
    worker = SerialWorker(args.port, args.baud)
    worker.start()

    for i in range(20):
        print(worker.queue)
        time.sleep(5)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", type=str, help="The serial port.")
    parser.add_argument("-b", "--baud", type=int, help="The baud rate.", default=115200)
    args = parser.parse_args()

    main()
