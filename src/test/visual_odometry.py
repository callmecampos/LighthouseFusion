from __future__ import print_function

from SuperPointPretrainedNetwork.demo_superpoint import *
from matplotlib import pyplot as plt
from matplotlib import animation as anim
import pyrealsense2 as rs
import numpy as np
import argparse, sys, os, cv2

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
    
    return qr, qx, qy, qz

def get_odometry(input_dir, focal_length, principal_pts):
    MIN_MATCH_COUNT = 10

    imgs = sorted(os.listdir(args.input))

    p, M_T = np.array([0, 0, 0, 1]).T, np.eye(4)

    with open(input_dir + '/../pose.freiburg', 'a+') as f:
        f.write("0 0 0 0 0 0 0 1\n")

    for i in range(0,len(imgs)):
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

        good = []
        pts1 = []
        pts2 = []

        # store all the good matches as per Lowe's ratio test.
        for m,n in matches:
            if m.distance < 0.8*n.distance:
                good.append(m)
                pts2.append(kp2[m.trainIdx].pt)
                pts1.append(kp1[m.queryIdx].pt)

        pts1 = np.float32(pts1)
        pts2 = np.float32(pts2)

        if len(good)>MIN_MATCH_COUNT:
            src_pts = pts1.reshape(-1,1,2)
            dst_pts = pts2.reshape(-1,1,2)

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

        E, mask = cv2.findEssentialMat(pts1, pts2,
                                    focal=focal_length, pp=principal_pts,
                                    method=cv2.RANSAC, prob=0.999, threshold=3.0)
        points, R, t, mask = cv2.recoverPose(E, pts1, pts2)
        R, t = np.array(R), np.array(t)

        T = np.hstack((np.vstack((R, np.zeros(3))), np.vstack((t, np.eye(1)))))
        p, M_T = np.dot(T, p), np.dot(M_T, T)

        # print(np.linalg.norm(np.dot(T, T.T) - np.eye(4)))
        # check matrix orthogonality before converting to quaternion representation
        try:
            x, y, z, u = p
            yaw, pitch, roll = rotation_to_ypr(M[:3][:3])
            qx, qy, qz, qw = ypr_to_quat(yaw, pitch, roll)
            with open(input_dir + '/../pose.freiburg', 'a') as f:
                f.write("{} {} {} {} {} {} {} {}\n".format(i+1, x, y, z, qx, qy, qz, qw))
        except:
            print("Matrix not orthogonal.")

        cv2.namedWindow('RGB F2F', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RGBF2F', cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params))
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

        print(i)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Feature Tracking for Visual Odometry.')
    parser.add_argument('input', type=str, default='', help='Image directory.')
    parser.add_argument('--intrinsics', type=str, default='/home/more3d/Dataset/intr/intrinsics.txt')
    args = parser.parse_args()
    print(args)

    fx, fy, ppx, ppy = open(args.intrinsics, 'r').read().split(' ')

    get_odometry(args.input, (float(fx)+float(fy))/2.0, (float(ppx), float(ppy)))
    
