import numpy as np
import cv2 as cv2
import argparse
import matplotlib.pyplot as plt
from turnRectStarter.py import sift_det
#added from sublime
numLeft = 0
numRight = 0
f = open("myfile.txt", "w")
# Construct the argument parser and parse the arguments
#ap = argparse.ArgumentParser()
# Image size is proportional to algorithm frame processing speed - bigger pictures cause more frame lag
# ap.add_argument("-i", 
#                 "--image", 
#                 required=True,
# 	            help="path to the static image that will be processed for keypoints")
# ap.add_argument("-l", 
#                 "--label",
#                 help="string to label the object found in the camera feed" )
# ap.add_argument("-s",
#                 "--source",
#                 type=int,
#                 help="optional argument for choosing a specific camera source (-1,0,1,2,3)")
# args = vars(ap.parse_args())

def matchLeftRightSift(image):
    #read in image from camera
    img = cv2.imread(image) 

    #get left and right sign images for matching with camera feed
    right = cv2.imread("./oneway.jpg")
    left = cv2.flip(right, 1)
    #plt.imshow(left),plt.show()
    #plt.imshow(right),plt.show()
    sift = cv2.xfeatures2d.SIFT_create()
# Labels the image as the name passed in    
# if args["label"] is not None:
#     label = args["label"]
# else:
#      Takes the name of the image as the name
#     if image[:2] == "./":
#         label = label = (image.split("/"))[2]
#     else:
#         label = image[2:-4]
#create SIFT detector
# find the keypoints and descriptors with ORB of all images
    kpImg, desImg = sift.detectAndCompute(img,None)
    kpLeft, desLeft = sift.detectAndCompute(left,None)
    kpRight, desRight = sift.detectAndCompute(right,None)
    bf = cv2.BFMatcher()
#get matches from brute force matcher for left sign
    leftMatches = bf.knnMatch(desImg,desLeft, k=2)
    rightMatches = bf.knnMatch(desImg, desRight, k=2)
# Apply ratio test on left matches
    leftGood = []
    for m,n in leftMatches:
        if m.distance < 0.75*n.distance:
            f.write("adding to left")
            leftGood.append([m])
# Apply ratio test on right matches
    rightGood = []
    for m,n in rightMatches:
        if m.distance < 0.75*n.distance:
            f.write("adding to right")
            rightGood.append([m])
    print("drawing images")
    img3 = cv2.drawMatchesKnn(img,kpImg,left,kpLeft,leftGood,None,flags=2)
    plt.imshow(img3),plt.show()
    numLeft = len(leftGood)
    numRight = len(rightGood)

def matchLeftRightTemplate(image):
    #read in image from camera
    img = cv2.imread(image)   
    #img = cv2.flip(img,1) 

    #get left and right sign images for matching with camera feed
    right = cv2.imread("./oneway.jpg")
    right = right[37:166, 288:410] #359205056.0
    left = cv2.flip(right, 1) #385317152

    signCoordinates = sift_det(image, "./oneway.jpg")[1]
    print(signCoordinates[0][1])
    img = img[signCoordinates[0][1]:signCoordinates[1][1], signCoordinates[0][0]:signCoordinates[1][0]]
    
    """
    for left test right template got 359205056.0 and left template got 385317152
    for right test right templat got __ and left template got 359204928.0
    """
    #img = img[200:430, 310:940]
    height, width, _ = img.shape
    rh, rw, _ = right.shape
    lh, lw, _ = left.shape

    rightMatch = cv2.matchTemplate(img,right,cv2.TM_CCOEFF)
    _, rightVal, _, leftLoc = cv2.minMaxLoc(rightMatch)

    leftMatch = cv2.matchTemplate(img,left,cv2.TM_CCOEFF)
    _, leftVal, _, rightLoc = cv2.minMaxLoc(leftMatch)
    print(leftVal)
    print(rightVal)
    
    if leftVal>rightVal:
        top_left = leftLoc
    else:
        top_left = rightLoc


    bottom_right = top_left[0] + lw, top_left[1] + lh

    cv2.rectangle(img,top_left, bottom_right, 255, 20)
    plt.subplot(121),plt.imshow(img,cmap = 'gray')
    plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(img,cmap = 'gray')
    plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
    if leftVal<rightVal:
        plt.subplot(121), plt.imshow(right)
        plt.title('Template'), plt.xticks([]), plt.yticks([])
    else:
        plt.subplot(121), plt.imshow(right)
        plt.title('Template'), plt.xticks([]), plt.yticks([])
    #plt.suptitle(meth)
    plt.show()
    return leftVal>rightVal

if __name__ == "__main__":
    print(matchLeftRightTemplate("./test1.jpeg"))
    #f = open("myfile.txt", "w")
    #f.write("left: " + str(numLeft) + "\n" + "right: " + str(numRight))
    f.close()
    #print(numLeft)
    #print(numRight)

