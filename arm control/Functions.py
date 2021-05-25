import math
import time
import numpy as np
from numpy import cos
from numpy import sin

from time import sleep

#--------------INITIAL ARM CONDITION
#arm length in cm
l1 = 12
l2 = 12.5
l3 = 18.3
l4 = 8.4-1.3

#angle limit for gamma
gamma_limit = 145*math.pi/180;

#FUNCTION FOR CALCULATING PSEUDO INVERSE OF JACOBIAN
def pseudo_J(theta, beta, gamma,l1, l2, l3):
    #form jacobian matrix
        J = np.matrix([[  sin(theta)*(l3*sin(beta + gamma) + l2*sin(beta)), -cos(theta)*(l3*cos(beta + gamma) + l2*cos(beta)), -l3*cos(beta + gamma)*cos(theta)],
                      [ -cos(theta)*(l3*sin(beta + gamma) + l2*sin(beta)), -sin(theta)*(l3*cos(beta + gamma) + l2*cos(beta)), -l3*cos(beta + gamma)*sin(theta)],
                      [                                                 0,             - l3*sin(beta + gamma) - l2*sin(beta),            -l3*sin(beta + gamma)]])
        
        #calculate pseudo inverse of jacobian matrix
        pseudo_J = np.linalg.pinv(J)

        return pseudo_J
#FUNCTION FOR CALCULATING END EFFECTOR LOCATION
def cart_cord_calibration(theta, beta, gamma): 
    f =  np.matrix([[-cos(theta)*(l3*sin(beta + gamma) + l2*sin(beta))],
                   [-sin(theta)*(l3*sin(beta + gamma) + l2*sin(beta))],
                   [         l1 + l3*cos(beta + gamma) + l2*cos(beta)]])  
      
    return np.array([f[0].item(), f[1].item(), f[2].item()-l4])

#FUNCTION FOR CALCULATING END EFFECTOR LOCATION
def cart_cord(theta, beta, gamma, l1, l2, l3): 
    f =  np.matrix([[-cos(theta)*(l3*sin(beta + gamma) + l2*sin(beta))],
                   [-sin(theta)*(l3*sin(beta + gamma) + l2*sin(beta))],
                   [         l1 + l3*cos(beta + gamma) + l2*cos(beta)]])  
      
    return np.array([f[0].item(), f[1].item(), f[2].item()])


#FUNCTION FOR RETURNING ANGLES WITH MAGNITUDE SMALLER THAN PI
def mod_angle(theta_ite):
  theta = theta_ite[0]
  beta = theta_ite[1]
  gamma = theta_ite[2]
  if theta>0 and theta>np.pi:
    theta = theta - 2*np.pi
  elif theta<0 and theta<-np.pi:
    theta = theta + 2*np.pi
  if beta>0 and beta>np.pi:
    beta = beta - 2*np.pi
  elif beta<0 and beta<-np.pi:
    beta = beta + 2*np.pi 
  if gamma>0 and gamma>np.pi:
    gamma = gamma - 2*np.pi
  elif gamma<0 and gamma<-np.pi:
    gamma = gamma + 2*np.pi

  return np.array([theta, beta, gamma])


def inv_kinematics(x_d, y_d, z_d, theta_c, beta_c, gamma_c):
    #current angles
    theta_ite = np.array([theta_c, beta_c, gamma_c])
    #desired position
    pos_desired = np.array([x_d, y_d, z_d])

    #initiate distance error
    pos_error = pos_desired - cart_cord(theta_ite[0], theta_ite[1], theta_ite[2], l1, l2, l3 )
    #initiate start time
    time_start_cal = time.perf_counter()
    while np.linalg.norm(pos_error)>0.001:
      inv_J = pseudo_J(theta_ite[0],theta_ite[1],theta_ite[2],l1,l2,l3 )
      del_angle = np.array(inv_J.dot(pos_error)).flatten()
      theta_ite = theta_ite + del_angle
      pos_error = pos_desired - cart_cord(theta_ite[0], theta_ite[1], theta_ite[2], l1, l2, l3 )
      
      #condition for jumping out from the loop
      if (time.perf_counter()-time_start_cal)>2:
          print("Solution can't be solved, original angle position is returned as answer")
          theta_ite = np.array([99*math.pi/180,99*math.pi/180 ,99*math.pi/180 ])
          break
    
    theta_ite = mod_angle(theta_ite%(2*np.pi))
    return theta_ite 
    

def time_scaled_path(angle_start,angle_end,T,t):
    s = (3/(T**2))*(t**2)-(2/(T**3))*(t**3) #calculate s for s domain, where theta = theta_start +s*(theta_end - theta_start)
    theta_desired = angle_start+s*(angle_end - angle_start)
    
    return theta_desired

def cal_calibration_end(x_position, y_position, z_position, theta_current, beta_current, gamma_current):
    execute = False #condition to start converting
    while execute == False:
        x_pos = float(x_position)
        y_pos = float(y_position)
        z_pos = float(z_position)
        z_pos = z_pos + l4
        
        theta_c_pseudo = np.arctan2(y_pos, x_pos)
        beta_c_pseudo = -math.pi/4
        gamma_c_pseudo = -math.pi/4
        
        if theta_c_pseudo > 2.618 or theta_c_pseudo < -2.618:
            theta_c_pseudo = 0 # for case that is out of the theta's range
            beta_c_pseudo = -beta_c_pseudo
            gamma_c_pseudo = -gamma_c_pseudo
            
        #calculate the desired angle positions corresponding to the entered position
        angle_desired = inv_kinematics(x_pos, y_pos, z_pos, theta_c_pseudo, beta_c_pseudo, gamma_c_pseudo)
        print("angle:", 180*angle_desired/math.pi)
        #three conditions on testing whether the calculated angle is within the operating range
        if angle_desired[0]>=-2.583 and angle_desired[0]<=2.583:
            if angle_desired[1]>=-math.pi/2 and angle_desired[1]<=math.pi/2:
                if angle_desired[2]>=-gamma_limit and angle_desired[2]<=gamma_limit:
                    execute == True
                    return angle_desired
                else:
                    print('angle gamma is not within bound')
                    return np.array([theta_current,beta_current,gamma_current])
            else:
                print('at least angle beta is not within bound')
                return np.array([theta_current,beta_current,gamma_current])
        else:
            print('at least angle theta is not within bound')
            return np.array([theta_current,beta_current,gamma_current])

def cal_angle_end():
    execute = False #condition to start converting
    while execute == False:
        x_pos = float(input('Desired x coordinate in cm: '))
        y_pos = float(input('Desired y coordinate in cm: '))
        z_pos = float(input('Desired z coordinate in cm: '))
        z_pos = z_pos + l4 #take consideration of the suction cup length
        check = input('Enter Y to confirm, any other key to reenter: ')
        if not(check == 'Y' or check == 'y'):
            break #break out from while loop to restart entering
        
        theta_c_pseudo = np.arctan2(y_pos, x_pos)
        beta_c_pseudo = -math.pi/4
        gamma_c_pseudo = -math.pi/4
        
        if theta_c_pseudo > 2.618 or theta_c_pseudo < -2.618:
            theta_c_pseudo = 0 # for case that is out of the theta's range
            beta_c_pseudo = -beta_c_pseudo
            gamma_c_pseudo = -gamma_c_pseudo
            
        #calculate the desired angle positions corresponding to the entered position
        angle_desired = inv_kinematics(x_pos, y_pos, z_pos, theta_c_pseudo, beta_c_pseudo, gamma_c_pseudo)
        print("angle:", 180*angle_desired/math.pi)
        #three conditions on testing whether the calculated angle is within the operating range
        if angle_desired[0]>=-2.583 and angle_desired[0]<=2.583:
            if angle_desired[1]>=-math.pi/2 and angle_desired[1]<=math.pi/2:
                if angle_desired[2]>=-gamma_limit and angle_desired[2]<=gamma_limit:
                    execute == True
                    return angle_desired
                else:
                    print('angle gamma is not within bound')
            else:
                print('at least angle beta is not within bound')
        else:
            print('at least angle theta is not within bound')
            
# class pump:
#     def __init__(self,dir1,dir2,pwm_pin):
#         self.dir1 = dir1
#         self.dir2 = dir2
#         self.pwm_pin = pwm_pin

#     def activate(self):
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(self.dir1,GPIO.OUT)
#         GPIO.output(self.dir1,GPIO.HIGH)
#         GPIO.setup(self.dir2,GPIO.OUT)
#         GPIO.output(self.dir2,GPIO.LOW)
#         GPIO.setup(self.pwm_pin,GPIO.OUT)
#         #CREATE A PWM OBJECT
#         self.p = GPIO.PWM(self.pwm_pin,1000)
#         self.p.start(0)
        
#     def deactivate(self):
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(self.dir1,GPIO.OUT)
#         GPIO.setup(self.dir2,GPIO.OUT)
#         GPIO.output(self.dir1,GPIO.LOW)
#         GPIO.output(self.dir2,GPIO.LOW)
#         GPIO.cleanup()
        
#     def pwm_value(self,pwm):
#         self.p.start(pwm)            
