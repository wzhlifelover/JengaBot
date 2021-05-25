import numpy as np
import cv2

# ZED Instrinsic parameters
# based on SN26462030.conf
# Options: 2K, FHD, HD, VGA
R, jacob = cv2.Rodrigues(np.array([0.0027161, 0, 0.000430528]))
T = np.array([[119.837, 0.0441432, -0.0386536]]).T

M1_2K = np.array([[1059.75, 0, 1118.19], 
                  [0, 1058.99, 615.079], 
                  [0, 0, 1]])



M1_HD = np.array([[529.875, 0, 645.595], 
                  [0, 529.495, 355.5395], 
                  [0, 0, 1]])


M2_2K = np.array([[1059.55, 0, 1147.19], 
                  [0, 1058.64, 634.958], 
                  [0, 0, 1]])

M2_HD = np.array([[529.775, 0, 660.095], 
                  [0, 529.32, 365.479], 
                  [0, 0, 1]])

# k1, k2, p1, p2, k3


dist1_2K = np.array([[-0.0435878, 0.0129137, -0.000229128, 0.000790854, -0.00592556]])
dist1_HD = np.array([[-0.0435878, 0.0129137, -0.000229128, 0.000790854, -0.00592556]])
dist2_2K = np.array([[-0.0398599, 0.00700217, -0.000241005, 0.000243961, -0.00359243]])
dist2_HD = np.array([[-0.0398599, 0.00700217, -0.000241005, 0.000243961, -0.00359243]])

M1 = M1_2K
M2 = M2_2K
dist1 = dist1_2K
dist2 = dist2_2K

image_size = (1920, 1080)

R1, R2, P1, P2, Q, ROI1, ROI2 = cv2.stereoRectify(M1, dist1, M2, dist2, image_size, R, T)

def triangulate_zed(x1, x2):
    x1 = cv2.undistortPoints(x1, M1, dist1, R=R1, P=P1)
    x2 = cv2.undistortPoints(x2, M2, dist2, R=R2, P=P2)

    X = cv2.triangulatePoints(P1, P2, x1, x2)
    X = X/X[3]
    X = X[0:3]

    return X
