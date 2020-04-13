'''
基于FLANN的匹配器(FLANN based Matcher)
1.FLANN代表近似最近邻居的快速库。它代表一组经过优化的算法，用于大数据集中的快速最近邻搜索以及高维特征。
2.对于大型数据集，它的工作速度比BFMatcher快。
3.需要传递两个字典来指定要使用的算法及其相关参数等
对于SIFT或SURF等算法，可以用以下方法：
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
对于ORB，可以使用以下参数：
index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12   这个参数是searchParam,指定了索引中的树应该递归遍历的次数。值越高精度越高
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
'''

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import os
import time
# start = time.time()
# queryImage=cv.imread("/home/stayalive/Documents/HERO/Reconstruction/python/l.jpg")
# trainingImage=cv.imread("/home/stayalive/Documents/HERO/Reconstruction/python/r.jpg")#读取要匹配的灰度照片
# queryImage = cv.resize(queryImage, (496, 372))
# trainingImage = cv.resize(trainingImage, (496, 372))
# cv.imshow("a", queryImage)
# cv.waitKey(30)

leftintrinsicmatrix = np.load("LeftIntrinsicMatrix.npy")
leftradialdistortion = np.load("LeftRadialDistortion.npy")
rightintrinsicmatrix = np.load("RightIntrinsicMatrix.npy")
rightradialdistortion = np.load("RightRadialDistortion.npy")
count = 0
cap = cv.VideoCapture(2)
while (True):
    ok, frame = cap.read()
    frame = cv.resize(frame, (1280, 480))
    if count == 4:
        os.system("./change.sh")
    if count != 5:
        count += 1
    if not ok:
        break
    queryImage = frame[0:480, 0:640, :]
    trainingImage = frame[0:480, 640:1280, :]
    sift=cv.xfeatures2d.SIFT_create()#创建sift检测器
    kp1, des1 = sift.detectAndCompute(queryImage,None)
    kp2, des2 = sift.detectAndCompute(trainingImage,None)
    #设置Flannde参数
    FLANN_INDEX_KDTREE=0
    indexParams=dict(algorithm=FLANN_INDEX_KDTREE,trees=5)
    searchParams= dict(checks=50)
    flann=cv.FlannBasedMatcher(indexParams,searchParams)
    matches=flann.knnMatch(des1,des2,k=2)
    #设置好初始匹配值
    matchesMask=[[0,0] for i in range (len(matches))]
    for i, (m,n) in enumerate(matches):
    	if m.distance< 0.5*n.distance: #舍弃小于0.5的匹配结果
    		matchesMask[i]=[1,0]
    drawParams=dict(matchColor=(0,0,255),singlePointColor=(255,0,0),matchesMask=matchesMask,flags=0) #给特征点和匹配的线定义颜色
    print(kp1[matches[0][0].queryIdx].pt)
    resultimage=cv.drawMatchesKnn(queryImage,kp1,trainingImage,kp2,matches,None,**drawParams) #画出匹配的结果
    cv.imshow("resultimage", resultimage)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
# stop = time.time()
# print(stop - start)
