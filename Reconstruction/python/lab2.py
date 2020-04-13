
from cv2 import *
import numpy as np
import os
import time

#start = time.time()
#left = imread("/home/stayalive/Documents/HERO/Reconstruction/python/l.jpg", 0)
#right = imread("/home/stayalive/Documents/HERO/Reconstruction/python/r.jpg", 0)
leftintrinsicmatrix = np.load("LeftIntrinsicMatrix.npy")
leftradialdistortion = np.load("LeftRadialDistortion.npy")
rightintrinsicmatrix = np.load("RightIntrinsicMatrix.npy")
rightradialdistortion = np.load("RightRadialDistortion.npy")
count = 0
cap = VideoCapture(2)
while (True):
    ok, frame = cap.read(0)
    frame = resize(frame, (1280, 480))
    if count == 4:
        os.system("./change.sh")
    if count != 5:
        count += 1
    if not ok:
        break
    left = frame[0:480, 0:640, :]
    right = frame[0:480, 640:1280, :]
    h, w = left.shape[:2]
    newcameramtx1, roi1 = getOptimalNewCameraMatrix(leftintrinsicmatrix, leftradialdistortion, (w, h), 1, (w, h))
    newcameramtx2, roi2 = getOptimalNewCameraMatrix(rightintrinsicmatrix, rightradialdistortion, (w, h), 1, (w, h))
    lmap1, lmap2 = initUndistortRectifyMap(leftintrinsicmatrix, leftradialdistortion, None, newcameramtx1, (w, h), 5)
    rmap1, rmap2 = initUndistortRectifyMap(rightintrinsicmatrix, rightradialdistortion, None, newcameramtx2, (w, h), 5)
    left = remap(left, lmap1, lmap2, INTER_LINEAR)
    right = remap(right, rmap1, rmap2, INTER_LINEAR)
    # left = blur(left, (3, 3))
    # right = blur(right, (3, 3))
    mindisparity = 0
    ndisparities = 64
    SADWindowSize = 13
    sgbm = StereoSGBM.create(mindisparity, ndisparities, SADWindowSize)
    P1 = int(8 * SADWindowSize * SADWindowSize)
    P2 = int(32 * SADWindowSize * SADWindowSize)
    sgbm.setP1(P1)
    sgbm.setP2(P2)
    sgbm.setPreFilterCap(15)
    sgbm.setUniquenessRatio(10)
    sgbm.setSpeckleRange(2)
    sgbm.setSpeckleWindowSize(100)
    sgbm.setDisp12MaxDiff(1)
    disp = sgbm.compute(left, right)
    disp = np.divide(disp, 16)
    #print(disp)
    disp8U = normalize(disp, 0, 255, CV_8UC1)
    #stop = time.time()
    #print(stop - start)
    imshow("sgbm", disp8U)
    imshow("left", left)
    imshow("right", right)
    if waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
destroyAllWindows()
