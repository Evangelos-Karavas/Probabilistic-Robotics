import vrep
import math
import numpy as np
import time
import sys

def vrep_connection():
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection

    if clientID!=-1:
        print ("Connected to remote API server")
    else:
        print("Not connected to remote API server")
        sys.exit("Could not connect")
    
    return clientID

def sample_normal_twelve(mu, sigma):
    # Sample from a normal distribution using 12 uniform samples
    # Formula returns sample from normal distribution with mean = 0
    x = 0.5 * np.sum(np.random.uniform(-sigma, sigma, 12))
    
    return mu + x


def get_robot_pose(clientID, robot_handle):
    
    err_code_pos, robot_pos = vrep.simxGetObjectPosition(clientID, robot_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_orient, robot_orient = vrep.simxGetObjectOrientation(clientID, robot_handle,-1 ,vrep.simx_opmode_blocking)
    robot_pose = np.array([robot_pos[0], robot_pos[1], robot_orient[2]])
    
    return robot_pose


def map_create(clientID):
    #Get landmarks/obstacles handles --> required for landmarks pose queries
    err_code, cuboid0_handle = vrep.simxGetObjectHandle(clientID,"Cuboid0", vrep.simx_opmode_blocking)
    err_code, cuboid1_handle = vrep.simxGetObjectHandle(clientID,"Cuboid1", vrep.simx_opmode_blocking)
    err_code, cuboid2_handle = vrep.simxGetObjectHandle(clientID,"Cuboid2", vrep.simx_opmode_blocking)
    err_code, cuboid3_handle = vrep.simxGetObjectHandle(clientID,"Cuboid3", vrep.simx_opmode_blocking)
    err_code, cuboid4_handle = vrep.simxGetObjectHandle(clientID,"Cuboid4", vrep.simx_opmode_blocking)
    err_code, cuboid5_handle = vrep.simxGetObjectHandle(clientID,"Cuboid5", vrep.simx_opmode_blocking)
    err_code, cuboid6_handle = vrep.simxGetObjectHandle(clientID,"Cuboid6", vrep.simx_opmode_blocking)
    err_code, cuboid7_handle = vrep.simxGetObjectHandle(clientID,"Cuboid7", vrep.simx_opmode_blocking)
    err_code, cuboid8_handle = vrep.simxGetObjectHandle(clientID,"Cuboid8", vrep.simx_opmode_blocking)
    err_code, cuboid9_handle = vrep.simxGetObjectHandle(clientID,"Cuboid9", vrep.simx_opmode_blocking)
    
    #Get landmarks/obstacles positions
    err_code_pos, cuboid0_pos = vrep.simxGetObjectPosition(clientID, cuboid0_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid1_pos = vrep.simxGetObjectPosition(clientID, cuboid1_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid2_pos = vrep.simxGetObjectPosition(clientID, cuboid2_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid3_pos = vrep.simxGetObjectPosition(clientID, cuboid3_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid4_pos = vrep.simxGetObjectPosition(clientID, cuboid4_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid5_pos = vrep.simxGetObjectPosition(clientID, cuboid5_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid6_pos = vrep.simxGetObjectPosition(clientID, cuboid6_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid7_pos = vrep.simxGetObjectPosition(clientID, cuboid7_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid8_pos = vrep.simxGetObjectPosition(clientID, cuboid8_handle,-1 ,vrep.simx_opmode_blocking)
    err_code_pos, cuboid9_pos = vrep.simxGetObjectPosition(clientID, cuboid9_handle,-1 ,vrep.simx_opmode_blocking)


    map_world = np.array( [cuboid0_pos[0:2], cuboid1_pos[0:2], cuboid2_pos[0:2], cuboid3_pos[0:2], cuboid4_pos[0:2], cuboid5_pos[0:2],
                           cuboid6_pos[0:2], cuboid7_pos[0:2], cuboid8_pos[0:2], cuboid9_pos[0:2]])

    return map_world

def read_sensor_data(initialCall, clientID, map_world, robot_pose):
    
    id_landmarks = []
    
    if initialCall:
        mode = vrep.simx_opmode_streaming
        initialCall = False
    else:
        mode = vrep.simx_opmode_buffer
    
    errorFlag,rawStringData=vrep.simxGetStringSignal(clientID,'landmarksPosition', mode) 
    returnCode = vrep.simxClearStringSignal(clientID, 'landmarksPosition', vrep.simx_opmode_oneshot)
    
    if errorFlag == vrep.simx_return_ok:
        rawFloatData=vrep.simxUnpackFloats(rawStringData)
        #print("New Scan Acquired:", rawFloatData)
        rawPosData = np.array([rawFloatData])
        id_landmarks = get_associated_landmarks(rawPosData, map_world, robot_pose)
    else:
        print('Measurements Awaiting')
        id_landmarks = []
        time.sleep(1.0)
        
    return initialCall, np.array(id_landmarks)

def get_associated_landmarks(rawPosData, map_world, robot_pose):
    
    dmap0 = math.sqrt(map_world[0,0]**2+map_world[0,1]**2)
    dmap1 = math.sqrt(map_world[1,0]**2+map_world[1,1]**2)
    dmap2 = math.sqrt(map_world[2,0]**2+map_world[2,1]**2)
    dmap3 = math.sqrt(map_world[3,0]**2+map_world[3,1]**2)
    dmap4 = math.sqrt(map_world[4,0]**2+map_world[4,1]**2)
    dmap5 = math.sqrt(map_world[5,0]**2+map_world[5,1]**2)
    dmap6 = math.sqrt(map_world[6,0]**2+map_world[6,1]**2)
    dmap7 = math.sqrt(map_world[7,0]**2+map_world[7,1]**2)
    dmap8 = math.sqrt(map_world[8,0]**2+map_world[8,1]**2)
    dmap9 = math.sqrt(map_world[9,0]**2+map_world[9,1]**2)
    
    
    
    rmap0 = math.atan2(map_world[0,1], map_world[0,0])
    rmap1 = math.atan2(map_world[1,1], map_world[1,0])
    rmap2 = math.atan2(map_world[2,1], map_world[2,0])
    rmap3 = math.atan2(map_world[3,1], map_world[3,0])
    rmap4 = math.atan2(map_world[4,1], map_world[4,0])
    rmap5 = math.atan2(map_world[5,1], map_world[5,0])
    rmap6 = math.atan2(map_world[6,1], map_world[6,0])
    rmap7 = math.atan2(map_world[7,1], map_world[7,0])
    rmap8 = math.atan2(map_world[8,1], map_world[8,0])
    rmap9 = math.atan2(map_world[9,1], map_world[9,0])
    
    rawPosData2c = np.reshape(rawPosData,(-1, 2))
    
    landmarks  = []
    
    d_off = 0.5
    r_off = 0.4
    
    for i in range (0, len(rawPosData2c)):
        
        dlm = math.sqrt(rawPosData2c[i,0]**2 + rawPosData2c[i,1]**2)
        rlm = math.atan2(rawPosData2c[i,1], rawPosData2c[i,0])
        
        landm_x = rawPosData2c[i,0]
        landm_y = rawPosData2c[i,1]
        
        bear = math.atan2((landm_y - robot_pose[1]), (landm_x - robot_pose[0])) - robot_pose[2]
        rng = math.sqrt((landm_x - robot_pose[0])**2 + (landm_y - robot_pose[1])**2 ) + 0.25
        
        if abs(dmap0 - dlm) < d_off and abs(rmap0 - rlm) < r_off:
           landmarks.append([0.0, rng, bear])
           
        if abs(dmap1 - dlm) < d_off and abs(rmap1 - rlm) < r_off:
           landmarks.append([1.0, rng, bear])
           
        if abs(dmap2 - dlm) < d_off and abs(rmap2 - rlm) < r_off:
           landmarks.append([2.0, rng, bear])
           
        if abs(dmap3 - dlm) < d_off and abs(rmap3 - rlm) < r_off:
           landmarks.append([3.0, rng, bear])
           
        if abs(dmap4 - dlm) < d_off and abs(rmap4 - rlm) < r_off:
           landmarks.append([4.0, rng, bear])
           
        if abs(dmap5 - dlm) < d_off and abs(rmap5 - rlm) < r_off:
           landmarks.append([5.0, rng, bear])
        
        if abs(dmap6 - dlm) < d_off and abs(rmap6 - rlm) < r_off:
           landmarks.append([6.0, rng, bear])
        
        if abs(dmap7 - dlm) < d_off and abs(rmap7 - rlm) < r_off:
           landmarks.append([7.0, rng, bear])
           
        if abs(dmap8 - dlm) < d_off and abs(rmap8 - rlm) < r_off:
           landmarks.append([8.0, rng, bear])
           
        if abs(dmap9 - dlm) < d_off and abs(rmap9 - rlm) < r_off:
           landmarks.append([9.0, rng, bear])
    
    #print ("Landmarks LIST:", landmarks)
    return landmarks

def set_motor_cmds (clientID, l_motor_handle, r_motor_handle, wl, wr):
    
    err_code = vrep.simxSetJointTargetVelocity(clientID,l_motor_handle, wl, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID,r_motor_handle, wr, vrep.simx_opmode_streaming)
    
    return
