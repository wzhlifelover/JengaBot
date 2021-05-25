import serial
import time
import math
import numpy as np
from Functions import cart_cord_calibration
from Functions import inv_kinematics
from Functions import cal_angle_end
from Functions import time_scaled_path
from Functions import cal_calibration_end
from Calibration import take_picture
from arm_camera_calibrate import get_cam2arm
from Calibration import check_calibration_error
from locate_jenga import get_jenga_location, get_jenga_orientation

#-----DEFINE VARIABLES
#DEFINE CURRENT ANGLES10
theta_current = 0

beta_current = -np.pi/4
gamma_current = -np.pi/2

#115200
ser=serial.Serial('COM5', baudrate=115200, timeout=1)
print('Waiting for Connection...')
time.sleep(10)



def write_three_angles(theta_desired,beta_desired,gamma_desired):
    #CONVERT RADIANT TO SERVO ANGLE SCALE
    theta_desired = int((theta_desired*180/math.pi)*(1023/300)+511.5)
    beta_desired = int((beta_desired*180/math.pi)*(1023/300)+511.5)
    gamma_desired = int((gamma_desired*180/math.pi)*(1023/300)+511.5)
    
    ser.write(b'u')
    theta = str(theta_desired)

    beta = str(beta_desired)

    gamma = str(gamma_desired)

    angle_out_str_encode = str.encode(theta+'*'+beta+'*'+gamma+'*')

    ser.write(angle_out_str_encode)
    
def read_three_angles():
    global theta_current
    global beta_current
    global gamma_current

    ser.write(b'x')
    theta_current = float(ser.readline().decode('ascii'))
    
 
    beta_current = float(ser.readline().decode('ascii'))

    gamma_current = float(ser.readline().decode('ascii'))

    theta_current = ((theta_current-511.5)*300/1023)*math.pi/180
    beta_current = ((beta_current-511.5)*300/1023)*math.pi/180
    gamma_current = ((gamma_current-511.5)*300/1023)*math.pi/180


def time_scaled_trace(angle_start, angle_end, T):
    time_start = time.perf_counter() #record the starting time of the path
    time_old = time_start #initiate old time for setting control frequency
    while (time.perf_counter()-time_start) < T:
        t = time.perf_counter()-time_start
        angle_desired = time_scaled_path(angle_start, angle_end, T, t)
        if (time.perf_counter() - time_old)>= 0.002:
            write_three_angles(angle_desired[0],angle_desired[1],angle_desired[2])
            time_old = time.perf_counter()

def write_suction_cup_state(pump_state,block_pos):
    
    ser.write(b'p')
    
    pump_state_str = str(pump_state)

    block_pos_str = str(block_pos)

    suction_cup_out_str_encode = str.encode(pump_state_str+'*'+block_pos_str+'*')

    ser.write(suction_cup_out_str_encode)
    
def take_calibration_photo(x_position,y_position,z_position,duration,picture_num):
    read_three_angles()
    angle_end = cal_calibration_end(x_position,y_position,z_position, theta_current,beta_current,gamma_current)
    T = duration
    angle_start = np.array([theta_current,beta_current,gamma_current]) #set the starting angle
    time_scaled_trace(angle_start, angle_end, T)
    #print(angle_start, angle_end)
    #wait for 2 sec before taking picture
    time.sleep(2)
    take_picture(picture_num+1)
    
    
#initiate calibration coordinate matirx
# point1 = np.array([20,0,10])
# point2 = np.array([10,0,10])
# point3 = np.array([10,-15,10])
# point4 = np.array([10,0,20])

point1 = np.array([13,7,2.3])
point2 = np.array([24,0,2.3])
point3 = np.array([13,-7,2.3])
point4 = np.array([17,0,16.3])

point_list = np.vstack((point1,point2,point3,point4))

def calibration():
    
    for count, point in enumerate(point_list):
      take_calibration_photo(point[0],point[1],point[2],1,count)  
    
def mov2cord(x,y,z):
    angle_end = cal_calibration_end(x,y,z, theta_current, beta_current, gamma_current)
    read_three_angles() #update angle configuration
    angle_start = np.array([theta_current,beta_current,gamma_current]) #set the starting angle
    time_scaled_trace(angle_start, angle_end, 1) #initiate path tracing

print('Start Actuation')
# create a pump object

start = input('Do you want to calibrate the camera and arm coordinate?(Y/N): ')
if start == 'y' or start == 'Y':
    tracker_present = 'n'
    while tracker_present == 'n' or tracker_present == 'N':
        calibration()
        tracker_present = input('Do all pictures contain the blue tracker (Y(skip)/N(retake)):')

cup_coord = np.hstack((np.array(point_list), np.ones((4, 1)))).T
# print(cup_coord)
cam2arm = get_cam2arm(cup_coord)
print("getting jenga location\n")
# center_jenga = get_jenga_location(cam2arm)
# print(center_jenga)

ser.flushInput()
auto_mode = input('Do you want to manually control arm (Y/N): ')
if auto_mode == 'Y' or auto_mode == 'y':
    while True:
        center_jenga, cord_pred_red, cord_pred_green, if_locate = get_jenga_location(cam2arm)
        print(center_jenga)
        if (if_locate != 'N' and if_locate != 'n'):
            angle_jenga = get_jenga_orientation(cord_pred_red, cord_pred_green)
            print(np.rad2deg(angle_jenga))
        angle_end = cal_angle_end() #prompt to ask for desired position and calculate corresponding angle configuration
        T = float(input('Input time duration to get to the desired position in s: ')) #prompt to ask for desired time duration for completing task
        read_three_angles() #update angle configuration
        angle_start = np.array([theta_current,beta_current,gamma_current]) #set the starting angle
        time_scaled_trace(angle_start, angle_end, T) #initiate path tracing
        pump_state = int(input('Pump State: '))
        block_pos = int(input('Block Position: '))
        write_suction_cup_state(pump_state,block_pos)
        #read_three_angles()
        #check_calibration_error(cart_cord_calibration(theta_current, beta_current, gamma_current), cam2arm)

if auto_mode == 'N' or auto_mode == 'n':
    
    jenga_stacked = 0
    while True:
        #get to initialized position
#         angle_end = cal_calibration_end(20, 0, 10, theta_current, beta_current, gamma_current)
#         read_three_angles() #update angle configuration
#         angle_start = np.array([theta_current,beta_current,gamma_current]) #set the starting angle
#         time_scaled_trace(angle_start, angle_end, 1) #initiate path tracing
        mov2cord(20,0,10)
        write_suction_cup_state(0,0)
        
        center_jenga, cord_pred_red, cord_pred_green, if_locate = get_jenga_location(cam2arm)
        print("center_jenga:", center_jenga)
        if (if_locate == 'Y' or if_locate == 'y'):
            angle_jenga = get_jenga_orientation(cord_pred_red, cord_pred_green)
            #print("jenga_angle ", np.rad2deg(angle_jenga))
        else:
            quit()
        
        
        #Get to Jenga Position
        
        
        mov2cord(center_jenga[0], center_jenga[1], 10)
        angle_end = cal_calibration_end(center_jenga[0], center_jenga[1], 2.0, theta_current, beta_current, gamma_current)
        
        #calculate angle changed
        jenga_grab_angle = angle_end[0]
        #print("jenga_grab_angle", jenga_grab_angle)
        jenga_stack_angle = -0.46364760900080615
        
        
        angle_rotate =  jenga_stack_angle - jenga_grab_angle + (np.pi/2)*(jenga_stacked%2) + angle_jenga
        angle_rotate = int(np.rad2deg(angle_rotate))
       # print("angle_rotate", angle_rotate)
        
        if angle_rotate <=0:
            block_pos = 180
            angle_rotate = 180+angle_rotate 
        else:
            block_pos = 0
            angle_rotate = angle_rotate
        write_suction_cup_state(0,block_pos)
        
        
        #angle_end = cal_angle_end() #prompt to ask for desired position and calculate corresponding angle configuration
        #T = float(input('Input time duration to get to the desired position in s: ')) #prompt to ask for desired time duration for completing task
        read_three_angles() #update angle configuration
        angle_start = np.array([theta_current,beta_current,gamma_current]) #set the starting angle
        time_scaled_trace(angle_start, angle_end, 1) #initiate path tracing
        if_correct_pos = input("Is the suction cup at the right position?(Y/N) ")
        #block_pos = int(input('Block Position: '))
        if (if_correct_pos == 'Y' or if_correct_pos == 'y'):
            #block_pos = 0 #need update angle
            write_suction_cup_state(1,block_pos)
        else:
            continue
        #read_three_angles()
        #check_calibration_error(cart_cord_calibration(theta_current, beta_current, gamma_current), cam2arm)
        
        #move up
        mov2cord(center_jenga[0], center_jenga[1], 15)

        #get to Jenga stack
        time.sleep(1)
        mov2cord(20, -10, 15)
        #jenga stack angle
#         jenga_stack_angle = angle_end[0]
#         print("jenga_stack_angle", jenga_stack_angle)
        

            
        write_suction_cup_state(1,angle_rotate)
        jenga_stacked+=1
        
        #place jenga down
        print("jengas in the stack", jenga_stacked)
        mov2cord(20, -10, 1.9*jenga_stacked)
        
        write_suction_cup_state(0,angle_rotate)
        time.sleep(0.5)
        
        mov2cord(20, -10, 15)
        write_suction_cup_state(0,0)