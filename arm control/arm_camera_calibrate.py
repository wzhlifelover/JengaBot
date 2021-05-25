# %matplotlib notebook

import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# thresholding parameters from Matlab
low_H = 0.621 * 180
low_S = 0.409 * 255
low_V = 0.235 * 255
high_H = 0.694 * 180
high_S = 0.947 * 255
high_V = 0.871 * 255

# find center of cup by color thresholding
def getCupLocation2D(fname):
    dir_name = "calibration_picture/"
    
    img = cv2.imread(dir_name + fname)

    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_threshold, 4, cv2.CV_32S)

    # Map component labels to hue val, 0-179 is the hue range in OpenCV
    label_hue = np.uint8(179*labels/np.max(labels))
    blank_ch = 255*np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

    # Converting cvt to BGR
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

    # set bg label to black
    labeled_img[label_hue==0] = 0
    
    # find component with largest area (excluding first component)
    cup_label = np.argmax(stats[1:,4]) + 1 

    return labeled_img, centroids[cup_label]

# Stereo Calibration with OpenCV
M1 = np.array([[1059.75, 0, 1118.19], 
                  [0, 1058.99, 615.079], 
                  [0, 0, 1]])


M2 = np.array([[1059.55, 0, 1147.19], 
                  [0, 1058.64, 634.958], 
                  [0, 0, 1]])

dist1 = np.array([[-0.0435878, 0.0129137, -0.000229128, 0.000790854, -0.00592556]])
dist2 = np.array([[-0.0398599, 0.00700217, -0.000241005, 0.000243961, -0.00359243]])

R, jacob = cv2.Rodrigues(np.array([0.0027161, 0, 0.000430528]))
T = np.array([[119.837, 0.0441432, -0.0386536]]).T

image_size = (1920, 1080)

R1, R2, P1, P2, Q, ROI1, ROI2 = cv2.stereoRectify(M1, dist1, M2, dist2, image_size, R, T)

def saveImage(labeled_img1, fname1, labeled_img2, fname2):
    # show image for down camera
    plt.imshow(cv2.cvtColor(labeled_img1, cv2.COLOR_BGR2RGB))
    plt.axis("off")
    plt.title("Down Image")
    plt.savefig("calibration_thresholding/" + fname1)
    
    # show image for up camera
    plt.imshow(cv2.cvtColor(labeled_img2, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.title("Up Image")
    plt.savefig("calibration_thresholding/" + fname2)

# triangulation of points
def triangulate(fname1, fname2, save=True, debug=True):
    labeled_img1, x1 = getCupLocation2D(fname1)
    labeled_img2, x2 = getCupLocation2D(fname2)

    x1 = cv2.undistortPoints(x1, M1, dist1, R=R1, P=P1)
    x2 = cv2.undistortPoints(x2, M2, dist2, R=R2, P=P2)

    X = cv2.triangulatePoints(P1, P2, x1, x2)
    X = X/X[3]
    X = X[0:3]
    
    if save:
        saveImage(labeled_img1, fname1, labeled_img2, fname2)
        
    if debug:
        print("threshold result for %s: " % fname1, end="")
        print(x1)
        print("threshold result for %s: " % fname2, end="")
        print(x2)
        
    return X

def get_cam2arm(cup_coord):
    # triangulate points
    print("begin calibration...")
    p1 = triangulate("down1.jpg", "up1.jpg")
    p2 = triangulate("down2.jpg", "up2.jpg")
    p3 = triangulate("down3.jpg", "up3.jpg")
    p4 = triangulate("down4.jpg", "up4.jpg")

    observed_points = np.hstack((p1, p2, p3, p4))
    observed_points = np.vstack((observed_points, np.ones((1,4))))
    
    # calculate transformation matrix
    cam2arm = np.matmul(cup_coord, np.linalg.inv(observed_points))
    
    print()
    print("calibration points in arm coordinate: ")
    print(cup_coord)
    
    print("calibration points in camera coordinate: ")
    print(observed_points)
    
    print("calibration matrix: ")
    print(cam2arm)
    
    print("finished calibrating")
    print()
    
    return cam2arm