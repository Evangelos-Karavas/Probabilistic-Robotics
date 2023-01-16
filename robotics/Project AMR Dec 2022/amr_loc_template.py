import keyboard
import math
import numpy as np
from numpy.linalg import inv 
import help_pkg


def keyboard_teleop():

    #Write code here
    u = 0
    w = 0  
    if keyboard.is_pressed('up'):
        u += 0.6
    if keyboard.is_pressed('down'):
        u -= 0.6
    if keyboard.is_pressed('left'):
        w += (12.0* 3.141529) / 180.0
    if keyboard.is_pressed('right'):
        w -= (12.0* 3.141529) / 180.0    
    return u, w


def servo_controller(u, w):
    
	#Write code here
    wl = (2.0* u - (w*0.381))/(2*0.111)
    wr = (2.0* u + (w*0.381))/(2*0.111)
    return wl, wr


def get_odometry_from_pose(pose, pose_prev, a):

    #Write code here
    delta_trans = math.sqrt((math.pow(pose[0] - pose_prev[0], 2)) + (math.pow(pose[1] - pose_prev[1], 2)))
    delta_rot_1 = math.atan2(pose[1] - pose_prev[1], pose[0] - pose_prev[0]) - (pose[2])
    delta_rot_2 = (pose[2] - pose_prev[2]) - (delta_rot_1)

    apoklisi_r1 = ((a[0]) * (abs(delta_rot_1))) + ((a[1]) * (abs(delta_trans)))
    apoklisi_r2 = ((a[2]) * (abs(delta_trans))) + ((a[3]) * (abs((delta_rot_1 + delta_rot_2))))
    apoklisi_r3 = ((a[0]) * (abs(delta_rot_2))) + ((a[1]) * (abs(delta_trans)))

    delta_hat_r1 = delta_rot_1 + help_pkg.sample_normal_twelve(0,apoklisi_r1)
    delta_hat_t = delta_rot_2 + help_pkg.sample_normal_twelve(0,apoklisi_r2)
    delta_hat_r2 = delta_trans + help_pkg.sample_normal_twelve(0,apoklisi_r3)

    return np.array([delta_hat_r1, delta_hat_t, delta_hat_r2])


def motion_model(odometry, robot_pose):

   #Write code here
    x_mm = robot_pose[0] + odometry[1] * math.cos(robot_pose[2] + odometry[0]) 
    y_mm = robot_pose[1] + odometry[1] * math.sin(robot_pose[2] + odometry[0]) 
    theta_mm = robot_pose[2] + odometry[0] + odometry[2]

    return np.array([x_mm, y_mm, theta_mm])


def ekf_algorithm(ekf_state, Sigma, robot_odometry, a, id_landmarks, map_world):
    
    #Write code here
    
    # #prediction
    # theta = ekf_state
    # G = np.array([[1.0, 0.0, -robot_odometry[1]*math.sin(theta + robot_odometry[0])], 
    #               [0.0, 1.0, robot_odometry[1]*math.cos(theta + robot_odometry[0])], 
    #               [0.0, 0.0, 1.0]])
    
    # V = np.array([[-robot_odometry[1]*math.sin(theta + robot_odometry[0]),math.cos(theta + robot_odometry[0]), 0.0], 
    #               [robot_odometry[1]*math.cos(theta + robot_odometry[0]), math.sin(theta + robot_odometry[0]), 0.0], 
    #               [1.0, 0.0,  1.0]])
    
    # M = np.array([[a[0]*(robot_odometry[0])**2 + a[1]*(robot_odometry[1])**2, 0.0, 0.0], 
    #               [0.0, a[2]*(robot_odometry[1])**2 + a[3]*((robot_odometry[0])**2 + (robot_odometry[2])**2), 0.0], 
    #               [0.0, 0.0, a[0]*(robot_odometry[0])**2 + a[1]*(robot_odometry[1])**2]])
    
    # ekf_state = ekf_state + np.array([[robot_odometry[1]*math.cos(theta + robot_odometry[0])], 
    #                                     [robot_odometry[1]*math.sin(theta + robot_odometry[0])], 
    #                                     [robot_odometry[0]+robot_odometry[2]]])
    
    # Sigma_next = G*Sigma*np.transpose(G) + V*M*np.transpose(V) 
    
    # #correction
    
    # z, measurements = help_pkg.read_sensor_data()
    # S = np.zeros(np.size(z,2),3, 3)
    # zHat = np.zeros(3,np.size(z,2))
    
    return ekf_state, Sigma