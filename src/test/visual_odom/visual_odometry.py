from __future__ import print_function

from SuperPointPretrainedNetwork.demo_superpoint import *
from matplotlib import pyplot as plt
from matplotlib import animation as anim
import pyrealsense2 as rs
import numpy as np
import argparse, sys, os, cv2

parser = argparse.ArgumentParser(description='Feature Tracking for Visual Odometry.')
parser.add_argument('input', type=str, default='',
  help='Image directory or movie file or "camera" (for webcam).')
#parser.add_argument('--weights_path', type=str, default='SuperPointPretrainedNetwork/superpoint_v1.pth',
#  help='Path to pretrained weights file (default: superpoint_v1.pth).')
#parser.add_argument('--img_glob', type=str, default='*.png',
#  help='Glob match if directory of images is specified (default: \'*.png\').')
#parser.add_argument('--skip', type=int, default=1,
#  help='Images to skip if input is movie or directory (default: 1).')
#parser.add_argument('--show_extra', action='store_true',
#  help='Show extra debug outputs (default: False).')
#parser.add_argument('--H', type=int, default=480,
#  help='Input image height (default: 480).')
#parser.add_argument('--W', type=int, default=640,
#  help='Input image width (default: 640).')
#parser.add_argument('--display_scale', type=int, default=2,
#  help='Factor to scale output visualization (default: 2).')
#parser.add_argument('--min_length', type=int, default=2,
#  help='Minimum length of point tracks (default: 2).')
#parser.add_argument('--max_length', type=int, default=5,
#  help='Maximum length of point tracks (default: 5).')
#parser.add_argument('--nms_dist', type=int, default=4,
#  help='Non Maximum Suppression (NMS) distance (default: 4).')
#parser.add_argument('--conf_thresh', type=float, default=0.2,
#  help='Detector confidence threshold (default: 0.2).')
#parser.add_argument('--nn_thresh', type=float, default=0.7,
#  help='Descriptor matching threshold (default: 0.7).')
#parser.add_argument('--camid', type=int, default=0,
#  help='OpenCV webcam video capture ID, usually 0 or 1 (default: 0).')
#parser.add_argument('--waitkey', type=int, default=1,
#  help='OpenCV waitkey time in ms (default: 1).')
#parser.add_argument('--cuda', action='store_true',
#  help='Use cuda GPU to speed up network processing speed (default: False)')
#parser.add_argument('--no_display', action='store_true',
#  help='Do not display images to screen. Useful if running remotely (default: False).')
#parser.add_argument('--write', action='store_true',
#  help='Save output frames to a directory (default: False)')
#parser.add_argument('--write_dir', type=str, default='tracker_outputs/',
#  help='Directory where to write output frames (default: tracker_outputs/).')
#parser.add_argument('--sift', action='store_true',
#  help='Option to use SIFT to find keypoints rather than SuperPoint.')
args = parser.parse_args()
print(args)

################################################3

MIN_MATCH_COUNT = 10

imgs = sorted(os.listdir(args.input))
homos = []

for i in range(0,300):
    if i+1 >= len(imgs):
        break
    f1, f2 = os.path.join(args.input, imgs[i]), os.path.join(args.input, imgs[i+1])

    img1 = cv2.imread(f1,0) # queryImage
    img2 = cv2.imread(f2,0) # trainImage

    sift = cv2.xfeatures2d.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)

        img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

    else:
        print("Not enough matches are found - {}/{}".format(len(good),MIN_MATCH_COUNT))
        matchesMask = None



    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)

    homos.append([plt.imshow(cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params))])

    print(i)

    #plt.imshow(img3, 'gray'),plt.show()

fig = plt.figure()
ani = anim.ArtistAnimation(fig, homos, interval=50, blit=True)
plt.show()
