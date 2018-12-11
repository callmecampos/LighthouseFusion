import serial, time, argparse

def serial_data(port, baudrate):
    ser = serial.Serial(port, baudrate)

    while True:
        yield ser.readline()

    ser.close()

parser = argparse.ArgumentParser()
parser.add_argument("port", type=str, help="The serial port.")
args = parser.parse_args()

for line in serial_data(args.port, 57600):
    data = line.decode("utf-8").split("\t")
    if data[0] == "ANG0":
        with open("angles.txt", "a+") as f:
            f.write(line.decode("utf-8") + str("\n"))
        continue
    tracking = len(data) == 7
    if tracking:
        print("Tracking...", end="\r")
        with open("poses.txt", "a+") as f:
            f.write("{} {} {}\n".format(data[3], data[4], data[5]))
    else:
        print("Not tracking...", end="\r")
