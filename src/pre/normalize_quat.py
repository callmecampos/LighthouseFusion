from math import sqrt

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = [n / mag for n in v]
    return v

with open("/home/more3d/lighthouse-slam/pilot/looptest1/pose.freiburg", 'r+') as f:
    lines = f.readlines()
    f.seek(0)
    f.truncate()

    init_quat = None
    for line in lines:
        data = line.split(" ")
        quats = data[4:8]
        if init_quat is None:
            init_quat = [float(q) for i, q in enumerate(quats) if i != 3]
            init_quat.extend([0])
        quats = [float(q) - init_quat[i] for i, q in enumerate(quats)]
        quats = normalize(quats)
        line = "{} {} {} {} {} {} {} {}\n".format(data[0], data[1], data[2], data[3], quats[0], quats[1], quats[2], quats[3])
        f.write(line)