import serial, time, argparse

def serial_data(port, baudrate):
    ser = serial.Serial(port, baudrate)

    while True:
        yield ser.readline()

    ser.close()

parser = argparse.ArgumentParser()
parser.add_argument("port", type=str, help="The serial port.")
parser.add_argument("-b", "--baud", type=int, help="The baud rate.", default=57600)
args = parser.parse_args()

i, p0 = 0, (0, 0, 0, 0)
for line in serial_data(args.port, args.baud):
    data = line.decode("utf-8").split("\t")
    if data[0] == "ANG0":
        with open("angles.txt", "a+") as f:
            f.write(line.decode("utf-8") + str("\n"))
        continue
    tracking = len(data) == 7
    if tracking:
        if i = 0:
            p0 = (float(data[3]), float(data[4]), float(data[5]), float(data[6]))
        pose = (float(data[3])-p0[0], float(data[4])-p0[1], float(data[5])-p0[2], float(data[6])-p0[3])
        print("Tracking ({})...".format(i), end="\r")
        with open("poses.txt", "a+") as f:
            f.write("{} {} {} {}".format(data[3], data[4], data[5], data[6]))
    else:
        print("Not tracking...", end="\r")
