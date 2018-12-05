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
import os, traceback, sys, argparse, json, serial, io

class cd:
    """Context manager for changing the current working directory"""
    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)



DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07"]

def find_device_that_supports_advanced_mode() :
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

def main():
    # Streaming loop
    valid_try = False
    ser= serial.Serial 
    ser.port= "/dev/ttyAM0" 
    ser.baudrate=500000
    ser.open()
    
    try:
        if not os.path.exists(args.directory):
            os.mkdir(args.directory)
            os.mkdir(args.directory+"/rgb/")
            os.mkdir(args.directory+"/depth/")
        else:
            if not os.path.exists(args.directory+"/rgb/"):
                os.mkdir(args.directory+"/rgb/")
            if not os.path.exists(args.directory+"/depth/"):
                os.mkdir(args.directory+"/depth/")
      
        # Create a pipeline
        pipeline = rs.pipeline()


        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()
        if not args.live: # if not running live, read from bag file
            rs.config.enable_device_from_file(config, args.input, False)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

        if args.live:
            #Enabling advanced mode to load settings from JSON file
            dev = find_device_that_supports_advanced_mode()
            advnc_mode = rs.rs400_advanced_mode(dev)
            print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

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
            with open(sys.path[0]+"/realsense_params.json") as f:
                as_json_object = json.load(f)
            if type(next(iter(as_json_object))) != str:
                as_json_object = {k.encode('utf-8'): v.encode("utf-8") for k, v in as_json_object.items()}
            # The C++ JSON parser requires double-quotes for the json object so we need
            # to replace the single quote of the pythonic json to double-quotes
            json_string = str(as_json_object).replace("'", '\"')
            advnc_mode.load_json(json_string)

        # Start streaming
        profile = pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 5 #1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        i, i0 = 0, 0
        ext = '.png' # '.jpg'
        t0 = time()
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image
            
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
            
            cv2.imwrite(os.path.join(args.directory + "/rgb/" + str(i).zfill(5) + ext), color_image)
            cv2.imwrite(os.path.join(args.directory + "/depth/" + str(i).zfill(5) + ext), depth_image)

            with open(args.directory + '/associations.txt', 'a+') as f:
                f.write(str(i) + ' ' + './depth/' + str(i).zfill(5) +ext+ ' ' + str(i) + ' ' + "./rgb/" + str(i).zfill(5) + ext+ "\n")
            
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
    finally:
        cv2.destroyAllWindows()
        if args.traceback:
            traceback.print_exc()

        if valid_try and not args.beta:
            with cd(sys.path[0] + '/png_to_klg/build'): # generate the .klg file
                os.system('./pngtoklg -w ' + args.directory + '/ -o ' + args.directory + '/realsense.klg')

            if args.elastic: # if flag enabled, run Elastic Fusion on the generated .klg
                print('Running ElasticFusion...')
                with cd(sys.path[0] + '/ElasticFusion/GUI/build'):
                    os.system('./ElasticFusion -l ' + args.directory + '/realsense.klg')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
    parser.add_argument("-i", "--input", type=str, help="Bag file to read")
    parser.add_argument("-e", "--elastic", help='Run ElasticFusion flag', action='store_true')
    parser.add_argument("-t", "--traceback", help='Show traceback flag', action='store_true')
    parser.add_argument("-l", "--live", help="Run on a live RealSense camera", action='store_true')
    parser.add_argument("-b", "--beta", help="Testing flag", action='store_true')

    args = parser.parse_args()

    main()

