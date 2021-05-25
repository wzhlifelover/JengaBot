import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import arm_camera_calibrate
import Calibration

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
    
def save1Image(labeled_img1, fname1):
    # show image for down camera
    plt.imshow(cv2.cvtColor(labeled_img1, cv2.COLOR_BGR2RGB))
    plt.axis("off")
    plt.title("Sample Image")
    plt.savefig("calibration_thresholding/" + fname1)
    

def getCupLocation2D_RED(fname):
    low_H1 = 0.992 * 180
    low_H2 = 0 * 180
    low_S = 0.585 * 255
    low_V = 0.572 * 255
    high_H1 = 1 * 180
    high_H2 = 0.057 * 180
    high_S = 0.746 * 255
    high_V = 0.859 * 255
    
    dir_name = "calibration_picture/"
    
    img = cv2.imread(dir_name + fname)

    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_threshold1 = cv2.inRange(img_HSV, (low_H1, low_S, low_V), (high_H1, high_S, high_V))
    img_threshold2 = cv2.inRange(img_HSV, (low_H2, low_S, low_V), (high_H2, high_S, high_V))
    img_threshold = img_threshold1 + img_threshold2
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_threshold, 4, cv2.CV_32S)

    # Map component labels to hue val, 0-179 is the hue range in OpenCV
    label_hue = np.uint8(179*labels/np.max(labels))
    blank_ch = 255*np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

    # Converting cvt to BGR
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

    # set bg label to black
    labeled_img[label_hue==0] = 0
    
    #e1Image(labeled_img, fname)
    
    # find component with largest area (excluding first component)
    cup_label = np.argmax(stats[1:,4]) + 1 

    return labeled_img, centroids[cup_label]



def getCupLocation2D_GREEN(fname):
    low_H = 0.282 * 180
    low_S = 0.358 * 255
    low_V = 0.424 * 255
    high_H = 0.390 * 180
    high_S = 0.579 * 255
    high_V = 0.655 * 255
    
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
    
    #ave1Image(labeled_img, fname)
    
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


    
    
# triangulation of points
def triangulate_RED(fname1, fname2, save=True, debug=True):
    labeled_img1, x1 = getCupLocation2D_RED(fname1)
    labeled_img2, x2 = getCupLocation2D_RED(fname2)

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

def triangulate_GREEN(fname1, fname2, save=True, debug=True):
    labeled_img1, x1 = getCupLocation2D_GREEN(fname1)
    labeled_img2, x2 = getCupLocation2D_GREEN(fname2)

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

def get_jenga_location(cam2arm):

#     labeled_img1, red_up = getCupLocation2D_RED('calibration_picture/up1.jpg')
#     labeled_img2, green_up = getCupLocation2D_GREEN('calibration_picture/up1.jpg')

#     labeled_img1, red_down = getCupLocation2D_RED('calibration_picture/down1.jpg')
#     labeled_img2, green_down = getCupLocation2D_GREEN('calibration_picture/down1.jpg')
#     plt.imshow(cv2.cvtColor(labeled_img1, cv2.COLOR_BGR2RGB))
#     plt.axis("off")
#     plt.title("Left Image")
#     plt.show()
    
#     plt.imshow(cv2.cvtColor(labeled_img2, cv2.COLOR_BGR2RGB))
#     plt.axis("off")
#     plt.title("right Image")
#     plt.show()
    if_locate = input("Do you want to locate Jenga(Y/N):")
    if(if_locate == 'N' or if_locate == 'n'):
        return "Not Locating Jenga",1,1, if_locate
    
    Calibration.take_picture(5)
    
    center_red = triangulate_RED('down5.jpg', 'up5.jpg', True, True)
    
    center_green = triangulate_GREEN('down5.jpg', 'up5.jpg', False, False)
    #print("after triang, before cam2arm", center_red, center_green)
    
    cord_pred_red = np.matmul(cam2arm, np.append(center_red, 1))
    cord_pred_red = cord_pred_red[0:3]/cord_pred_red[3]
    #print("red:", cord_pred_red)
    
    cord_pred_green = np.matmul(cam2arm, np.append(center_green, 1))
    cord_pred_green = cord_pred_green[0:3]/cord_pred_green[3]
    #print("green:", cord_pred_green)
    
    center_jenga = (cord_pred_red + cord_pred_green)/2
    
    return center_jenga, cord_pred_red, cord_pred_green, if_locate

print(get_jenga_location)

def get_jenga_orientation(cord_pred_red, cord_pred_green):
    vec_jenga = cord_pred_green[0:2] - cord_pred_red[0:2]
    x1 = vec_jenga[0]
    y1 = vec_jenga[1]
    x2 = 1
    y2 = 0
    angle = np.arccos((x1*x2 + y1*y2)/np.linalg.norm(vec_jenga))
    return angle