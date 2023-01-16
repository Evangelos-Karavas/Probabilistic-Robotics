#import necessary packages
import vrep # access all the VREP elements
import numpy as np # package for linear algebra
from matplotlib import pyplot as plt #package for plot
from matplotlib.animation import FuncAnimation
import help_pkg #help functions that accompany the project
import amr_loc_template
#Initiliazing connection with CoppeliaSim (vrep) via RemoteAPI in Python language
clientID = help_pkg.vrep_connection()
#####################################################################################################  

#Create map of the environment
#map_world array contains the position [x_lan, y_lan] of all landmarks. 
#In case of n landmarks the dimension is n x 2 
map_world = help_pkg.map_create(clientID)
#####################################################################################################

#Get useful handles from CoppeliaSim (vrep)
#Get mobile robot handle --> required for pose queries
err_code, robot_handle = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx", vrep.simx_opmode_blocking)

#Get motors handles --> required for sending commands to the wheel motors
err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx_leftMotor", vrep.simx_opmode_blocking)
err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx_rightMotor", vrep.simx_opmode_blocking)
######################################################################################################

#Get robot's initial pose 
#robot_pose_prev contains the initial [x, y, theta] pose of the robot
#at the end of each iteration robot_pose_prev = robot_pose
robot_pose_prev = help_pkg.get_robot_pose(clientID, robot_handle)
#####################################################################################################

#state_mm is the state of the robot estimated ONLY from the motion (odometry model).
#It is used only for comparive reasons.
#We initialize with the pose given by the simulator:
state_mm = robot_pose_prev
######################################################################################################

#id_landmarks array contains the detected landmarks at each code iteration.
#if m landmarks are visible the dimension will be m x 3.
#Each raw: 1st element is landmark id e.g. 1.0, 2.0, 3.0,...Second and third elements are landmark's  
#range and bearing with respect to the robot
#We initialize as below, but the dimension will be formulated dynamically according to the detected landmarks
#at each cycle. This is handled by function: get_associated_landmarks
id_landmarks = np.array([[0.0, 0.0, 0.0]])
######################################################################################################

#Boolean variable to check if it is the first time we ask the sensor for measurements
#it is required for the proper communication with the sensor in coppeliasim (vrep)
initialCall=True
######################################################################################################

#Initialize the ekf state
#ekf_state contains the estimates of the robot's pose [x, y, theta]
ekf_state = np.array([robot_pose_prev[0], robot_pose_prev[1], robot_pose_prev[2]])

#Initialize the ekf covariance matrix
Sigma = 10.0*np.array([[0.01, 0.0, 0.0], 
                  [0.0, 0.01, 0.0], 
                  [0.0, 0.0, 0.01]])
######################################################################################################

#Odometry noise parameters
a = np.array([0.1, 0.1, 0.01, 0.01])
######################################################################################################

#This is the main loop of the code
while True:
    #Get robot's current pose
    #This is the actual robot pose.
    #It is used to calculate odometry and as ground truth to check the performance of the EKF.
    robot_pose = help_pkg.get_robot_pose(clientID, robot_handle)
    
    #Get measurements from sensor
    #See above for the description of id_landmarks
    initialCall, id_landmarks = help_pkg.read_sensor_data(initialCall, clientID, map_world, robot_pose)
    print("ID LandMarks", id_landmarks)
    
    #Call keyboard teleoperation function here
    u, w = amr_loc_template.keyboard_teleop()
    
    #Call servo_controller function here
    wl, wr = amr_loc_template.servo_controller(u, w)
    
    #Set motor commands (rotational velocities) (uncomment when wl, wr are available)
    help_pkg.set_motor_cmds (clientID, l_motor_handle, r_motor_handle, wl, wr)
    
    #Calculate odometry inputs from current - previous poses and noise a
    odometry = amr_loc_template.get_odometry_from_pose(robot_pose,robot_pose_prev,a)
    
    robot_odometry = amr_loc_template.motion_model(odometry, robot_pose)

    #Estimate robot's pose via EKF algorithm
    ekf_state = amr_loc_template.ekf_algorithm(ekf_state, Sigma, robot_odometry, a, id_landmarks, map_world)
    
    #At the end of each iteration we assign the current actual robot pose to the previous one, robot_pose_prev.
    #We need this for the odometry calculation
    robot_pose_prev = robot_pose