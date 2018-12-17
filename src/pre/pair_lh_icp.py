icp_quat = []
with open("/home/more3d/lighthouse-slam/pilot/meter4test/realsense.klg.freiburg", "r") as f:
    lines = f.readlines()

    for line in lines:
        data = line.split(" ")
        quats = data[4:8]
        icp_quat.append(quats)

with open("/home/more3d/lighthouse-slam/pilot/meter4test/pose.freiburg", 'r+') as f:
    lines = f.readlines()
    f.seek(0)
    f.truncate()

    init_quat = None
    for i, line in enumerate(lines):
        if i == len(icp_quat):
            break
        data = line.split(" ")
        quats = icp_quat[i]
        line = "{} {} {} {} {} {} {} {}".format(data[0], data[1], data[2], data[3], quats[0], quats[1], quats[2], quats[3])
        f.write(line)