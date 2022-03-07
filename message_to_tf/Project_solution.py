#################################################################################
# This is the solution to project one. This script or part of the script should #
# not be shared to any student prior to Lab completion                          #
#################################################################################

import cv2
import numpy as np

###### SECTION 1: DETECTION OF KEYPOINTS ##############################

def Detectors(image, algo=None):

    assert algo is not None, "Please specify the detection algorithm you want to use. This can be 'brisk', 'sift', 'surf' and 'orb'"

    if algo == 'brisk':
        descriptor = cv2.BRISK_create()
    
    elif algo == 'sift':
        descriptor = cv2.SIFT_create()
    
    elif algo == 'surf':
        descriptor  = cv2.xfeatures2d.SURF_create()

    elif algo == 'orb':
        descriptor = cv2.ORB_create()

    
    (kps, features) = descriptor.detectAndCompute(image, None)
    
    return (kps, features)

##################### SECTION 1.2: DRAWING FEATURES #############
def Drawer(image, kp, color):
    if color == 'red':
        Img_ = cv2.drawKeypoints(image, kp, outImage=None, color=(0,0,255), flags = 0)
    
    elif color == 'green':
        Img_ = cv2.drawKeypoints(image, kp, outImage=None, color=(0,255,0), flags = 0)
    
    elif color == 'green':
        Img_ = cv2.drawKeypoints(image, kp, outImage=None, color=(255,0,0), flags = 0)
   
    return Img_

##################### SECTION 2: FEATURES MATCHING #############

##################### SECTION 2.1: USING BRUTE-FORCE #############
def FeatureMatch(img1,img2,algo,crossCheck,kp1,des1,kp2,des2):
    
    
    if algo == 'sift' or algo == 'surf':
        bf = cv2.BFMatcher()


    elif algo == 'orb' or algo == 'brisk':
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=crossCheck)

    matches = bf.knnMatch(des1,des2,k=2)

    # Apply ratio test for matches
    good = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
          good.append(m)

    # Drawing matches
    #img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,outImg=None,flags=2)

    return (good)

############### SECTION 2.2: USING FLANN METHOD ##############
###############################################################

def FeatureMatchFlan(img1,img2,kp1,des1,kp2,des2):
    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)
    matchesMask = [[0,0] for i in range(len(matches))]

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
            
    draw_params = dict(matchColor = (0,255,0),
                    singlePointColor = (255,0,0),
                    matchesMask = matchesMask,
                    flags = 0)

    img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)
    
    return (img3, good)

    

######### SECTION 3: HOMOGRAPHY ESTIMATION ################################
def homography(img1,img2,kp1,kp2,good,MIN):
    if len(good)>MIN:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        (M, mask) = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        (h,w,z) = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
    
    else:
        print ('Not enough matches are found - %d/%d', (len(good),MIN))
        matchesMask = None
    
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

    img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
    return (img3, M, dst)
    
############################## SECTION 4: WARPING ################
def warping(img1, img2, M):
    img3 =cv2.warpPerspective(img1,M,(img1.shape[1]+img2.shape[0],img2.shape[0]))
    img3[0:img2.shape[0], 0:img2.shape[1]] = img2
    return img3
#############################################################################
#    MAIN BODY
###########################################################################
#Read images
img2 =cv2.imread('IMG_1879.jpg')
img1 = cv2.imread('IMG_1880.jpg')

#img1 =cv2.imread('IMG_1884.jpg')
#img2 = cv2.imread('IMG_1885.jpg')

#img1 =cv2.imread('result.jpg')
#img2 = cv2.imread('IMG_1886.jpg')


img1 = cv2.resize(img1,(1000,1000),interpolation=cv2.INTER_AREA)

img2 = cv2.resize(img2,(1000,1000),interpolation=cv2.INTER_AREA)
#############

(kp1, des1) = Detectors(img1, 'sift')
(kp2, des2) = Detectors(img2, 'sift')

points = Drawer(img1, kp1, 'red')
points1 = Drawer(img2,kp2,'green')


cv2.imshow('Detected points image 1', points)
cv2.waitKey(0)

cv2.imshow('Detected points image 2', points1)
cv2.waitKey(0)

(matches) = FeatureMatch(img1,img2,'sift', True, kp1,des1,kp2,des2)
#(matched_image, matches) = FeatureMatchFlan(img1,img2,kp1,des1,kp2,des2)


(Homographed_img, M, sss) = homography(img1,img2,kp1,kp2,matches, 5)
cv2.imshow('Homogra[hy', Homographed_img)
cv2.waitKey(0)

results = warping(img1, img2, M)
cv2.imshow('result', results)
cv2.waitKey(0)

cv2.imwrite('result.jpg', results)
cv2.destroyAllWindows()



    


