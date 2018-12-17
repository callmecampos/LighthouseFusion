# native python stuff
from __future__ import print_function
from time import time

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# General OS, parameter parsing, and debugging
import os, sys, serial, traceback, argparse, json, collections, shutil

''' MARK: OS & Serial '''

class cd:
    """Context manager for changing the current working directory"""
    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)

def serial_data(port, baud):
    ser = serial.Serial(port, baud)
    val = ser.readline().decode("utf-8")
    ser.close()
    return time(), val

''' MARK: RealSense '''

DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07"]

def find_device_that_supports_advanced_mode():
    ctx = rs.context()
    ds5_dev = rs.device()
    devices = ctx.query_devices();
    print(devices)
    for dev in devices:
        print(dev.supports(rs.camera_info.product_id))
        print(str(dev.get_info(rs.camera_info.product_id)))
        if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            if dev.supports(rs.camera_info.name):
                print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
            return dev
    raise Exception("No device that supports advanced mode was found")

''' MARK: Quaternion Math '''

# Reference: http://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf

def rotation_to_ypr(R):
    pitch = np.arctan2(-R[2][0], np.sqrt(R[0][0] + R[1][0]))

    # degenerate case
    if np.abs(pitch) == 90:
        sign = np.sign(pitch)
        return np.arctan2(sign * R[1][2], sign * R[0][2]), pitch, 0

    return np.arctan2(R[1][0], R[0][0]), pitch, np.arctan2(R[2][1], R[2][2])

def ypr_to_quat(y, p, r):
    qr = np.cos(r/2.)*np.cos(p/2.)*np.cos(y/2.) + np.sin(r/2.)*np.sin(p/2.)*np.sin(y/2.)
    qx = np.sin(r/2.)*np.cos(p/2.)*np.cos(y/2.) - np.cos(r/2.)*np.sin(p/2.)*np.sin(y/2.)
    qy = np.cos(r/2.)*np.sin(p/2.)*np.cos(y/2.) + np.sin(r/2.)*np.cos(p/2.)*np.sin(y/2.)
    qz = np.cos(r/2.)*np.cos(p/2.)*np.sin(y/2.) - np.sin(r/2.)*np.sin(p/2.)*np.cos(y/2.)
    
    return qx, qy, qz, qr

''' MARK: Main '''

def main():
    # Streaming loop
    valid_try = False
    arr = []
    depth_scale = 0.0010000000475
    try:
        if args.live or args.input:
            assert args.input or (args.live and not os.path.exists(args.directory)), "Output directory already exists."

            os.mkdir(args.directory)
            os.mkdir(args.directory+"/rgb/")
            os.mkdir(args.directory+"/depth/")

            os.mkdir(args.directory+"/tmp/") # necessary to achieve full FPS
            os.mkdir(args.directory+"/tmp/rgb/")
            os.mkdir(args.directory+"/tmp/depth/")

            # Create a pipeline
            pipeline = rs.pipeline()

            # Create a config and configure the pipeline to stream
            #  different resolutions of color and depth streams
            config = rs.config()
            if not args.live: # if not running live, read from bag file
                rs.config.enable_device_from_file(config, args.input, False)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, args.fps)

            if args.live:
                # Enabling advanced mode to load settings from JSON file
                dev = find_device_that_supports_advanced_mode()
                advnc_mode = rs.rs400_advanced_mode(dev)
                print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

                # set FPS in presets JSON file
                filename = args.preset + '.json'
                with open(filename, "r+") as jsonFile:
                    data = json.load(jsonFile, object_pairs_hook=collections.OrderedDict)

                    tmp = data["stream-fps"]
                    data["stream-fps"] = str(args.fps)

                    jsonFile.seek(0)  # rewind
                    json.dump(data, jsonFile, indent=4)
                    jsonFile.truncate()

                # Loop until we successfully enable advanced mode
                while not advnc_mode.is_enabled():
                    print("Trying to enable advanced mode...")
                    advnc_mode.toggle_advanced_mode(True)
                    # At this point the device will disconnect and re-connect.
                    print("Sleeping for 5 seconds...")
                    time.sleep(5)
                    # The 'dev' object will become invalid and we need to initialize it again
                    dev = find_device_that_supports_advanced_mode()
                    advnc_mode = rs.rs400_advanced_mode(dev)
                    print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

                # Loading controls from a json string
                # For Python 2, the values in 'as_json_object' dict need to be converted from unicode object to utf-8
                with open(sys.path[0]+"/high_accuracy.json") as f:
                    as_json_object = json.load(f)
                if type(next(iter(as_json_object))) != str:
                    as_json_object = {k.encode('utf-8'): v.encode("utf-8") for k, v in as_json_object.items()}
                # The C++ JSON parser requires double-quotes for the json object so we need
                # to replace the single quote of the pythonic json to double-quotes
                json_string = str(as_json_object).replace("'", '\"')
                advnc_mode.load_json(json_string)

            # Start streaming
            profile = pipeline.start(config)

            # Getting camera intrinsics and loading to disk
            depth_stream = profile.get_stream(rs.stream.depth)
            depth_intr = depth_stream.as_video_stream_profile().get_intrinsics()
            with open(args.directory + '/intrinsics.txt','w') as f:
                f.write("{} {} {} {}".format(depth_intr.fx, depth_intr.fy, depth_intr.ppx, depth_intr.ppy)) # load camera intrinsics

            # Getting the depth sensor's depth scale (see rs-align example for explanation)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            print("Depth Scale is: " , depth_scale)

            # We will be removing the background of objects more than
            #  clipping_distance_in_meters meters away
            clipping_distance_in_meters = 3
            clipping_distance = clipping_distance_in_meters / depth_scale

            # Create an align object
            # rs.align allows us to perform alignment of depth frames to others frames
            # The "align_to" is the stream type to which we plan to align depth frames.
            align_to = rs.stream.color
            align = rs.align(align_to)

            i, i0 = 0, 0
            pose, ext = None, '.png' # '.jpg'

            t0 = time()
            while i < 1200:
                # Get frameset of color and depth
                frames = pipeline.wait_for_frames()
                if not frames:
                    continue

                t_prev = time()
                if args.serial:
                    data = serial_data(args.port, args.baud)
                    if (data[0] - t_prev > 0.1): # if pose is old
                        print("Old pose at frame {}, skipping.".format(i))
                        continue
                    pose = data[1].split("\t")
                    if len(pose) != 12:
                        print("Serial data not complete at frame {}, skipping.".format(i))
                        pose = arr[-1][2]
                    else:
                        if pose[0] == "Setup:" or pose[0] == "Error:":
                            print("Pose estimation not setup yet. Breaking.")
                            break

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    print('Frame ' + str(i) + ' not aligned correctly.')
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # save our numpy arrays to a tmp directory so we can maintain our FPS
                #    and write the images to disk later
                outfile_color = args.directory + "/tmp/rgb/{}.npy".format(i)
                outfile_depth = args.directory + "/tmp/depth/{}.npy".format(i)
                np.save(outfile_color, color_image)
                np.save(outfile_depth, depth_image)

                arr.append([outfile_depth, outfile_color, pose])

                if (False):
                    # Remove background - Set pixels further than clipping_distance to grey
                    grey_color = 153
                    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
                    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

                    # Render images
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    images = np.hstack((bg_removed, depth_colormap))
                    cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('Align Example', images)
                    key = cv2.waitKey(1)
                    # Press esc or 'q' to close the image window
                    if key & 0xFF == ord('q') or key == 27:
                        cv2.destroyAllWindows()
                        break
                i += 1

                print('Running frame-rate avg.: {} Hz ({})'.format(float(i-i0) / (time()-t0), i), end='\r')

                if (i % 20 == 0): # reset running average frame-rate params
                    t0, i0 = time(), i

                valid_try = True
        else:
            valid_try = True
            int('aaron rodgers tha goat')
    finally:
        cv2.destroyAllWindows()

        print()

        if not valid_try or args.traceback:
            traceback.print_exc()

        if args.live or args.input:
            print('Writing RGB and depth channel pairs to disk...')

            init_pose = None
            for i, data in enumerate(arr):
                # load the numpy arrays from disk
                df, cf, pose = data
                depth_image = np.load(df)
                color_image = np.load(cf)

                # delete the tmp files
                os.remove(df)
                os.remove(cf)

                # write the images to disk
                cv2.imwrite(os.path.join(args.directory + "/rgb/" + str(i).zfill(5) + ext), color_image)
                cv2.imwrite(os.path.join(args.directory + "/depth/" + str(i).zfill(5) + ext), depth_image)

                if i == 0:
                    init_pose = (float(pose[4]), float(pose[6]), float(pose[5]), float(pose[8]), float(pose[9]), float(pose[10]))

                # update the RGB-Depth associations file
                with open(args.directory + '/associations.txt', 'a+') as f:
                    f.write(str(i) + ' ' + './depth/' + str(i).zfill(5) + ext + ' ' + str(i) + ' ' + "./rgb/" + str(i).zfill(5) + ext + "\n")
                    
                quat = ypr_to_quat(float(pose[8])-init_pose[3], float(pose[9])-init_pose[4], float(pose[10])-init_pose[5])

                with open(args.directory + '/pose.freiburg', 'a+') as f:
                    f.write("{} {} {} {} {} {} {} {}\n".format(i, float(pose[4])-init_pose[0], float(pose[6])-init_pose[1], float(pose[5])-init_pose[2],
                                                            quat[0], quat[1], quat[2], quat[3]))

                print('Progress: {}/{} pairs saved.'.format(i, len(arr)), end='\r')

            print()

        if valid_try and not args.to_png:
            print("Merging RGB and Depth images into ElasticFusion compatible format.")

            with cd(sys.path[0] + '/../../png_to_klg/build'): # generate the .klg file
                os.system('./pngtoklg -w ' + args.directory + '/ -o ' + args.directory + '/realsense.klg -t -s 1250')

            if args.elastic: # if flag enabled, run Elastic Fusion on the generated .klg
                print('Running ElasticFusion...')
                with cd(sys.path[0] + '/../../ElasticFusion/GUI/build'):
                    if args.ground:
                        os.system('./ElasticFusion -l ' + args.directory + '/realsense.klg -d 8 -f -cal ' + args.directory + '/intrinsics.txt -p ' + args.directory + '/pose.freiburg')
                    else:
                        os.system('./ElasticFusion -l ' + args.directory + '/realsense.klg -d 8 -f -cal ' + args.directory + '/intrinsics.txt')

        if args.delete and args.live:
            shutil.rmtree(args.directory)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory", type=str, help="Path to store outputs (images and klg, possible also mesh?)")
    parser.add_argument("-i", "--input", type=str, help="Bag file to read")
    parser.add_argument("-e", "--elastic", help='Run ElasticFusion flag', action='store_true')
    parser.add_argument("-t", "--traceback", help='Show traceback flag', action='store_true')
    parser.add_argument("-l", "--live", help="Run on a live RealSense camera", action='store_true')
    parser.add_argument("-g", "--ground", help="Run ElasticFusion on ground truth estimates.", action='store_true')
    parser.add_argument("-s", "--serial", help="Run multicore process to read from serial.", action='store_true')
    parser.add_argument("-p", "--port", type=str, help="The serial port.")
    parser.add_argument("-b", "--baud", type=int, help="The baud rate.", default=115200)
    parser.add_argument("--to_png", help="Testing flag", action='store_true')
    parser.add_argument("--fps", type=int, help="frame rate to run the camera at", default=30)
    parser.add_argument("--preset", type=str, help="RealSense camera presets", default='high_accuracy')
    parser.add_argument("--delete", help="Delete the dataset directory after running.", action='store_true')

    args = parser.parse_args()

    assert args.fps == 15 or args.fps == 30 or args.fps == 60 or args.fps == 90, "An invalid FPS was provided, supported rates are: 15, 30, 60, 90"
    assert os.path.isfile(args.preset + ".json"), "Presets file does not exist."
    assert not (args.live and args.input), "Incompatible device input settings (choose live camera or .bag)"

    main()
