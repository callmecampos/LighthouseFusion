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
import os, traceback, sys, argparse

class cd:
    """Context manager for changing the current working directory"""
    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)

def main():
    # Streaming loop
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
        rs.config.enable_device_from_file(config, args.input, False)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

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

        i = 0
        ext = '.png' # '.jpg'
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
    finally:
        cv2.destroyAllWindows()
        if args.traceback:
            traceback.print_exc()

        with cd(sys.path[0] + '/png_to_klg/build'): # generate the .klg file
            os.system('./pngtoklg -w ' + args.directory + '/ -o ' + args.directory + '/realsense.klg')

        if args.elastic: # if flag enabled, run Elastic Fusion on the generated .klg
            with cd(sys.path[0] + '/ElasticFusion/GUI/build'):
                os.system('./ElasticFusion -l ' + args.directory + '/realsense.klg')
        #pipeline.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
    parser.add_argument("-i", "--input", type=str, help="Bag file to read")
    parser.add_argument("-e", "--elastic", help='Run ElasticFusion flag', action='store_true')
    parser.add_argument("-t", "--traceback", help='Show traceback flag', action='store_true')
    args = parser.parse_args()

    main()

