import os
from arm_camera_calibrate import triangulate
import numpy as np
import pyzed.sl as sl





def take_picture(picture_num):
#     cameraUp_select()
#     camera.capture('calibration_picture/up%d.jpg' % picture_num)
#     cameraDown_select()
#     camera.capture('calibration_picture/down%d.jpg' % picture_num)
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.camera_fps = 30  # Set fps at 30
    #print("Opening the Camera\n")
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns SUCCESS
        #LEFT
        zed.retrieve_image(image, sl.VIEW.LEFT)
        img = image.get_data()
        img_name = 'calibration_picture/up{}.jpg'.format(picture_num)
        image.write(img_name)
        
        #RIGHT
        zed.retrieve_image(image, sl.VIEW.RIGHT)
        img = image.get_data()
        img_name = 'calibration_picture/down{}.jpg'.format(picture_num)
        image.write(img_name)

        

def check_calibration_error(cord_actual, cam2arm):
#     cameraUp_select()
#     camera.capture('calibration_picture/up_sample.jpg')
#     cameraDown_select()
#     camera.capture('calibration_picture/down_sample.jpg')
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.camera_fps = 30  # Set fps at 30
    #print("Opening the Camera\n")
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns SUCCESS
        #LEFT
        zed.retrieve_image(image, sl.VIEW.LEFT)
        img = image.get_data()
        img_name = 'calibration_picture/up_sample.jpg'
        image.write(img_name)
        
        #RIGHT
        zed.retrieve_image(image, sl.VIEW.RIGHT)
        img = image.get_data()
        img_name = 'calibration_picture/down_sample.jpg'
        image.write(img_name)

    X = triangulate('down_sample.jpg', 'up_sample.jpg')
    print('tracker position in camera coordinate:')
    print(' (%1.3f, %1.3f, %1.3f)' % (X[0], X[1], X[2]))
    
    cord_pred = np.matmul(cam2arm, np.append(X, 1))
    cord_pred = cord_pred[0:3]/cord_pred[3]
    
    print('tracker position in arm coordinate:')
    print('using dynamixel - (%1.3f, %1.3f, %1.3f)' % (cord_actual[0], cord_actual[1], cord_actual[2]))
    print('using camera - (%1.3f, %1.3f, %1.3f)' % (cord_pred[0], cord_pred[1], cord_pred[2]))
    print('error - (%f)' % np.linalg.norm(cord_actual - cord_pred))
