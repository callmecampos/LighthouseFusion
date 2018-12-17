import serial, time, argparse

def serial_data(port, baud):
    t0 = time.time()
    ser = serial.Serial(port, baud)
    t1 = time.time()
    print("opening: {}".format(t1-t0))
    val = time.time(), ser.readline().decode("utf-8")
    t0 = time.time()
    ser.close()
    t1 = time.time()
    print("closing: {}".format(t1-t0))
    return val

parser = argparse.ArgumentParser()
parser.add_argument("port", type=str, help="The serial port.")
parser.add_argument("-b", "--baud", type=int, help="The baud rate.", default=57600)
args = parser.parse_args()

i, p0 = 0, (0, 0, 0, 0)



while True:
    print(serial_data(args.port, args.baud))
