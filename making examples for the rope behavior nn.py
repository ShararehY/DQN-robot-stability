try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import numpy
import math
import random
import sys
#sys.path.append(r'C:\Program Files\V-REP3\V-REP_PRO_EDU')
import transforms3d

#math functions
def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))


#function: defining the stopping criteria
def StopSimulation(clientID,PS1handle,PS2handle,PS3handle,PJ1handle,PJ2handle,PJ3handle):
    #PS1=proximity_sensor_1 and PJ1=prismatic_joint_1

    isStopped=0
    #--reading the distance handle data
    resPS1,detectionState1,_,_,_=vrep.simxReadProximitySensor(clientID,PS1handle,vrep.simx_opmode_buffer)
    if resPS1!=vrep.simx_return_ok:
        print ('Cannot read proximity sensor 1')

    resPS2,detectionState2,_,_,_=vrep.simxReadProximitySensor(clientID,PS2handle,vrep.simx_opmode_buffer)
    if resPS2!=vrep.simx_return_ok:
        print ('Cannot read proximity sensor 2')

    resPS3,detectionState3,_,_,_=vrep.simxReadProximitySensor(clientID,PS3handle,vrep.simx_opmode_buffer)
    if resPS3!=vrep.simx_return_ok:
        print ('Cannot read proximity sensor 3')

    #--reading the joint handle forces
    resPJ1,joint_force_1=vrep.simxGetJointForce(clientID,PJ1handle,vrep.simx_opmode_buffer)
    if resPJ1!=vrep.simx_return_ok:
        print ('Cannot read force of prismatic joint 1:',resPJ1)
   
    resPJ2,joint_force_2=vrep.simxGetJointForce(clientID,PJ2handle,vrep.simx_opmode_buffer)
    if resPJ1!=vrep.simx_return_ok:
        print ('Cannot read force of prismatic joint 2:',resPJ2)

    resPJ3,joint_force_3=vrep.simxGetJointForce(clientID,PJ3handle,vrep.simx_opmode_buffer)
    if resPJ1!=vrep.simx_return_ok:
        print ('Cannot read force of prismatic joint 3:',resPJ3)

    #--stopping the simulation if all the data is being recieved and the robot has flipped or the cables are torn
    if resPS1==vrep.simx_return_ok and resPS2==vrep.simx_return_ok and resPS3==vrep.simx_return_ok and resPJ1==vrep.simx_return_ok and resPJ2==vrep.simx_return_ok \
       and resPJ3==vrep.simx_return_ok:
        if detectionState1==0 or detectionState2==0 or detectionState3==0:
            print('Robot flipped. Simulation stopped.')
            # Stoping the Simulation
            res=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
            isStopped=1
            

        if  abs(joint_force_1)>200 or abs(joint_force_2)>200 or abs(joint_force_3)>200:
            print('Cable is torn. Simulation stopped.')
            # Stoping the Simulation
            res=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
            isStopped=1
            
    return isStopped
            

#function: moving
def move(ClientID,inputs,name):
    #inputs=[V_rightWheel,V_leftWheel,V_rightWheel#0,V_leftWheel#0,V_rightWheel#1,V_leftWheel#1,V_prismatic_1,V_prismatic_2,V_prismatic_3]
    #if a value in input array does not change (is the same as before), put 's' instead of a number
    
    if type(inputs[6]) is not str:
        #assign new velocity for joint
        res=vrep.simxSetJointTargetVelocity(ClientID,prismatic_joint_1,inputs[6],vrep.simx_opmode_oneshot_wait)
        if res!=vrep.simx_return_ok:
            print('error in setting rope 1 velocity')
        

    if type(inputs[7]) is not str:
        #assign new velocity for joint
        res=vrep.simxSetJointTargetVelocity(ClientID,prismatic_joint_2,inputs[7],vrep.simx_opmode_oneshot_wait)
        if res!=vrep.simx_return_ok:
            print('error in setting rope 2 velocity')
        

    if type(inputs[8]) is not str:
        #assign new velocity for joint
        res=vrep.simxSetJointTargetVelocity(ClientID,prismatic_joint_3,inputs[8],vrep.simx_opmode_oneshot_wait)
        if res!=vrep.simx_return_ok:
            print('error in setting rope 3 velocity')
        
    if name=='Pioneer#1':
        if type(inputs[5]) is not str:
            res=vrep.simxSetJointTargetVelocity(ClientID,P1_leftWheel,inputs[5],vrep.simx_opmode_streaming)
            if res !=vrep.simx_return_ok:
                print ('error in moving joint P1_leftWheel')

        if type(inputs[4]) is not str:
            res=vrep.simxSetJointTargetVelocity(ClientID,P1_rightWheel,inputs[4],vrep.simx_opmode_streaming)
            if res !=vrep.simx_return_ok:
                print ('error in moving joint P1_rightWheel')

    if name=='Pioneer#0':
        if type(inputs[3]) is not str:
            res=vrep.simxSetJointTargetVelocity(ClientID,P0_leftWheel,inputs[3],vrep.simx_opmode_streaming)
            if res !=vrep.simx_return_ok:
                print ('error in moving joint P0_leftWheel')

        if type(inputs[2]) is not str:
            res=vrep.simxSetJointTargetVelocity(ClientID,P0_rightWheel,inputs[2],vrep.simx_opmode_streaming)
            if res !=vrep.simx_return_ok:
                print ('error in moving joint P0_rightWheel')

    if name=='Pioneer':
        if type(inputs[1]) is not str:
            res=vrep.simxSetJointTargetVelocity(ClientID,P_leftWheel,inputs[1],vrep.simx_opmode_streaming)
            if res !=vrep.simx_return_ok:
                print ('error in moving joint P_leftWheel')

        if type(inputs[0]) is not str:
            res=vrep.simxSetJointTargetVelocity(ClientID,P_rightWheel,inputs[0],vrep.simx_opmode_streaming)
            if res !=vrep.simx_return_ok:
                print ('error in moving joint P_rightWheel')



#function: find an object's z axis in the main coordinations
def zAxis(Euler):
    quant=transforms3d.euler.euler2quat(Euler[0],Euler[1],Euler[2],axes='sxyz')
    vector=transforms3d.quaternions.rotate_vector([0,0,1],quant)
    return vector
def yAxis(Euler):
    quant=transforms3d.euler.euler2quat(Euler[0],Euler[1],Euler[2],axes='sxyz')
    vector=transforms3d.quaternions.rotate_vector([0,1,0],quant)
    return vector

#function: find an object's x axis in the main coordinations
def xAxis(Euler):
    quant=transforms3d.euler.euler2quat(Euler[0],Euler[1],Euler[2],axes='sxyz')
    vector=transforms3d.quaternions.rotate_vector([1,0,0],quant)
    return vector


#function: get all the inputs for one robot
def NNinput(robotName):
    #inputs: 1)tilt=90-gama  2)right wheel velocity  3)left wheel velocity
    #        4)right rope tension  5)left rope tension  6)right rope up-down angle
    #        7)left rope up-down angle  8)right rope left-right angle
    #        9)left rope left-right angle

    #robotName: 'Pioneer' or 'Pioneer#0' or 'Pioneer#1'

    if robotName=='Pioneer':
        rightWheel = P_rightWheel
        leftWheel = P_leftWheel
        UpperDiskJoint = P_UpperDiskJoint
        left_joint = prismatic_joint_1
        right_joint = prismatic_joint_2
        robot = pioneer
    if robotName == 'Pioneer#0':
        rightWheel = P0_rightWheel
        leftWheel = P0_leftWheel
        UpperDiskJoint = P0_UpperDiskJoint
        left_joint = prismatic_joint_3
        right_joint = prismatic_joint_2
        robot = pioneer0
    if robotName == 'Pioneer#1':
        rightWheel = P1_rightWheel
        leftWheel = P1_leftWheel
        UpperDiskJoint = P1_UpperDiskJoint
        left_joint = prismatic_joint_3
        right_joint = prismatic_joint_1
        robot = pioneer1

    # getting the wheel velocities
    res,rightWheel_V=vrep.simxGetObjectFloatParameter(clientID,rightWheel,2012,vrep.simx_opmode_blocking)
    res,leftWheel_V=vrep.simxGetObjectFloatParameter(clientID,leftWheel,2012,vrep.simx_opmode_blocking)

    #getting the tilt angle
    res,euler=vrep.simxGetObjectOrientation(clientID,UpperDiskJoint,-1,vrep.simx_opmode_blocking)
    z=zAxis(euler)
    tilt=(numpy.pi/2)-angle(z,[0,0,1])
        
    #getting rope tensions
    res,leftRopeTension=vrep.simxGetJointForce(clientID,left_joint,vrep.simx_opmode_buffer)
    res,rightRopeTension=vrep.simxGetJointForce(clientID,right_joint,vrep.simx_opmode_buffer)
        
    #getting ropes' up-down angle (90-angle with pioneer coordinates' z axis)
    #and left-right angles (angle with pioneer coordinates' x axis)
    res,euler=vrep.simxGetObjectOrientation(clientID,left_joint,robot,vrep.simx_opmode_blocking) #euler relative
    # to pioneer coordinate system
    z=zAxis(euler)
    leftRopeUDAngle=(numpy.pi/2)-angle(z,[0,0,1])
    leftRopeLRAngle=angle([z[0],z[1]],[1,0])
        
    res,euler=vrep.simxGetObjectOrientation(clientID,right_joint,robot,vrep.simx_opmode_blocking) #euler 
    # relative to pioneer coordinate system
    z=zAxis(euler)
    rightRopeUDAngle=(numpy.pi/2)-angle(z,[0,0,1])
    rightRopeLRAngle=angle([z[0],z[1]],[1,0])

    res, position =  vrep.simxGetObjectPosition(clientID, pioneer, -1, vrep.simx_opmode_blocking)
    res, position0 = vrep.simxGetObjectOrientation(clientID, pioneer0, -1, vrep.simx_opmode_blocking)
    res, position1 = vrep.simxGetObjectOrientation(clientID, pioneer1, -1, vrep.simx_opmode_blocking)
    x,y,z=position
    x0, y0, z0 = position0
    x1, y1, z1 = position1

    d0=pow(x-x0,2)+pow(y-y0,2)+pow(z-z0,2)
    d1=pow(x-x1,2)+pow(y-y1,2)+pow(z-z1,2)
    d2=pow(x0-x1,2)+pow(y0-y1,2)+pow(z0-z1,2)

    return [tilt,rightWheel_V,leftWheel_V,rightRopeTension,leftRopeTension,rightRopeUDAngle,leftRopeUDAngle,rightRopeLRAngle,leftRopeLRAngle,d0,d1,d2]


#--- main program -------------------------------------------------------------------------------------
EPISODES = 15   #how many times to start from a new starting point
start_points=17 #how many starting point files we have

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    fileNum=26
    #!!---------loading desired scene---------
    #for episodeNum in range(10,EPISODES+4):
    for episodeNum in range(1):
        #make completely sure sim is stopped
        res=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        time.sleep(5)
        #close current scene
        res=vrep.simxCloseScene(clientID,vrep.simx_opmode_oneshot_wait)
        if res==vrep.simx_return_ok:
            print('Previous file successfully closed.')
        else:
            print('file did not close with error code:',res)
            
        #name of file
        #fileNum = fileNum + 1
        fileNum=12

        print('File number:    ',fileNum)
        name='D:\\internship\\robot final files\\final\\samples\\1\\test'+str(fileNum)+'.ttt'
        
        res=vrep.simxLoadScene(clientID,name,0,vrep.simx_opmode_blocking)
        if res==0:
            print('file successfully loaded.')

        #vrep.simxSynchronous(clientID,True);

        
        ##--- getting object handles ---
        #--getting proximity sensor handles
        res,proximity_sensor_1=vrep.simxGetObjectHandle(clientID,'Proximity_sensor_1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('proximity sensor 1 handle recieved.')
            #initializing the stream that reads sensor
            res,_,_,_,_=vrep.simxReadProximitySensor(clientID,proximity_sensor_1,vrep.simx_opmode_streaming)
        else:
            print ('getting proximity sensor 1: call returned with error code: ',res)

        res,proximity_sensor_2=vrep.simxGetObjectHandle(clientID,'Proximity_sensor_2',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('proximity sensor 2 handle recieved.')
            #initializing the stream that reads sensor
            res,_,_,_,_=vrep.simxReadProximitySensor(clientID,proximity_sensor_2,vrep.simx_opmode_streaming)
        else:
            print ('getting proximity sensor 2: call returned with error code: ',res)
        
        res,proximity_sensor_3=vrep.simxGetObjectHandle(clientID,'Proximity_sensor_3',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('proximity sensor 3 handle recieved.')
            #initializing the stream that reads sensor
            res,_,_,_,_=vrep.simxReadProximitySensor(clientID,proximity_sensor_3,vrep.simx_opmode_streaming)
        else:
            print ('getting proximity sensor 3: call returned with error code: ',res)

        #--getting prismatic joint handles
        res,prismatic_joint_1=vrep.simxGetObjectHandle(clientID,'Prismatic_joint_1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('prismatic joint 1 handle recieved.')
            #initializing the stream that reads force of joint
            res,_=vrep.simxGetJointForce(clientID,prismatic_joint_1,vrep.simx_opmode_streaming)
            res,_=vrep.simxGetJointPosition(clientID,prismatic_joint_1,vrep.simx_opmode_streaming)
        else:
            print ('getting prismatic joint 1: call returned with error code: ',res)

        res,prismatic_joint_2=vrep.simxGetObjectHandle(clientID,'Prismatic_joint_2',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('prismatic joint 2 handle recieved.')
            #initializing the stream that reads force of joint
            res,_=vrep.simxGetJointForce(clientID,prismatic_joint_2,vrep.simx_opmode_streaming)
            res,_=vrep.simxGetJointPosition(clientID,prismatic_joint_2,vrep.simx_opmode_streaming)
        else:
            print ('getting prismatic joint 2: call returned with error code: ',res)

        res,prismatic_joint_3=vrep.simxGetObjectHandle(clientID,'Prismatic_joint_3',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('prismatic joint 3 handle recieved.')
            #initializing the stream that reads force of joint
            res,_=vrep.simxGetJointForce(clientID,prismatic_joint_3,vrep.simx_opmode_streaming)
            res,_=vrep.simxGetJointPosition(clientID,prismatic_joint_3,vrep.simx_opmode_streaming)
        else:
            print ('getting prismatic joint 3: call returned with error code: ',res)
        
        #--getting wheel handles
        res,P_leftWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer left wheel handle recieved.')
            #initializing the stream that reads position of the wheel
            res,_=vrep.simxGetJointPosition(clientID,P_leftWheel,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer left wheel: call returned with error code:',res)

        res,P_rightWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer right wheel handle recieved.')
            #initializing the stream that reads position of the wheel
            res,_=vrep.simxGetJointPosition(clientID,P_rightWheel,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer right wheel: call returned with error code:',res)

        res,P0_leftWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#0',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#0 left wheel handle recieved.')
            #initializing the stream that reads position of the wheel
            res,_=vrep.simxGetJointPosition(clientID,P0_leftWheel,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#0 left wheel: call returned with error code:',res)
        
        res,P0_rightWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#0',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#0 right wheel handle recieved.')
            #initializing the stream that reads position of the wheel
            res,_=vrep.simxGetJointPosition(clientID,P0_rightWheel,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#0 right wheel: call returned with error code:',res)

        res,P1_leftWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#1 left wheel handle recieved.')
            #initializing the stream that reads position of the wheel
            res,_=vrep.simxGetJointPosition(clientID,P1_leftWheel,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#1 left wheel: call returned with error code:',res)

        res,P1_rightWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#1 right wheel handle recieved.')
            #initializing the stream that reads position of the wheel
            res,_=vrep.simxGetJointPosition(clientID,P1_rightWheel,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#1 right wheel: call returned with error code:',res)

        #--getting spherical joint handles
        res,P_leftSphericalJoint=vrep.simxGetObjectHandle(clientID,'Spherical_left',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer left spherical joint handle recieved.')
            #initializing the stream that reads position of the spherical joint
            res,_=vrep.simxGetJointMatrix(clientID,P_leftSphericalJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer left spherical joint: call returned with error code:',res)

        res,P_rightSphericalJoint=vrep.simxGetObjectHandle(clientID,'Spherical_right',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer right spherical joint handle recieved.')
            #initializing the stream that reads position of the spherical joint
            res,_=vrep.simxGetJointMatrix(clientID,P_rightSphericalJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer right spherical joint: call returned with error code:',res)

        res,P0_leftSphericalJoint=vrep.simxGetObjectHandle(clientID,'Spherical_left#0',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#0 left spherical joint handle recieved.')
            #initializing the stream that reads position of the spherical joint
            res,_=vrep.simxGetJointMatrix(clientID,P0_leftSphericalJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#0 left spherical joint: call returned with error code:',res)
        
        res,P0_rightSphericalJoint=vrep.simxGetObjectHandle(clientID,'Spherical_right#0',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#0 right spherical joint handle recieved.')
            #initializing the stream that reads position of the spherical joint
            res,_=vrep.simxGetJointMatrix(clientID,P0_rightSphericalJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#0 right spherical joint: call returned with error code:',res)

        res,P1_leftSphericalJoint=vrep.simxGetObjectHandle(clientID,'Spherical_left#1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#1 left spherical joint handle recieved.')
            #initializing the stream that reads position of the spherical joint
            res,_=vrep.simxGetJointMatrix(clientID,P1_leftSphericalJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#1 left spherical joint: call returned with error code:',res)

        res,P1_rightSphericalJoint=vrep.simxGetObjectHandle(clientID,'Spherical_right#1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#1 right spherical joint handle recieved.')
            #initializing the stream that reads position of the spherical joint
            res,_=vrep.simxGetJointMatrix(clientID,P1_rightSphericalJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting Pioneer#1 right spherical joint: call returned with error code:',res)

        #--getting the revolute upper disk joint handles
        res,P_UpperDiskJoint=vrep.simxGetObjectHandle(clientID,'Revolute_upperDisk',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('revolute upper disk joint handle recieved.')
            #initializing the stream that reads joint orientation (for tilt)
            res,_=vrep.simxGetObjectOrientation(clientID,P_UpperDiskJoint,-1,vrep.simx_opmode_streaming)
            #initializing the stream that reads joint position
            res,_=vrep.simxGetJointPosition(clientID,P_UpperDiskJoint,vrep.simx_opmode_streaming)        
        else:
            print ('getting revolute upper disk joint: call returned with error code: ',res)

        res,P0_UpperDiskJoint=vrep.simxGetObjectHandle(clientID,'Revolute_upperDisk#0',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('revolute upper disk joint#0 handle recieved.')
            #initializing the stream that reads joint orientation (for tilt)
            eulerAngle=vrep.simxGetObjectOrientation(clientID,P0_UpperDiskJoint,-1,vrep.simx_opmode_streaming)
            #initializing the stream that reads joint position
            res,_=vrep.simxGetJointPosition(clientID,P0_UpperDiskJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting revolute upper disk joint#0: call returned with error code: ',res)
        
        res,P1_UpperDiskJoint=vrep.simxGetObjectHandle(clientID,'Revolute_upperDisk#1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('revolute upper disk joint#1 handle recieved.')
            #initializing the stream that reads joint orientation (for tilt)
            eulerAngle=vrep.simxGetObjectOrientation(clientID,P1_UpperDiskJoint,-1,vrep.simx_opmode_streaming)
            #initializing the stream that reads joint position
            res,_=vrep.simxGetJointPosition(clientID,P1_UpperDiskJoint,vrep.simx_opmode_streaming)
        else:
            print ('getting revolute upper disk joint#1: call returned with error code: ',res)

        #getting the robots' object handles
        res,pioneer=vrep.simxGetObjectHandle(clientID,'Pioneer',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer recieved.')        
        else:
            print ('getting pioneer: call returned with error code: ',res)

        res,pioneer0=vrep.simxGetObjectHandle(clientID,'Pioneer#0',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#0 recieved.')        
        else:
            print ('getting pioneer#0: call returned with error code: ',res)

        res,pioneer1=vrep.simxGetObjectHandle(clientID,'Pioneer#1',vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('pioneer#1 recieved.')        
        else:
            print ('getting pioneer#1: call returned with error code: ',res)
        

        ##--- starting the simulation -------------------------------------------------------------------------------------
        res=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        time.sleep(3)
        if res==vrep.simx_return_ok:
            print ('Simulation started')
        else:
            print ('Starting Simulation call returned with error code: ',res)
        
        ##------------------------------------TEMP
        #getting prismatic joint forces to see if they are possitive or not
        res,pris1Force=vrep.simxGetJointForce(clientID,prismatic_joint_1,vrep.simx_opmode_buffer)
        print(pris1Force)
        if res!=vrep.simx_return_ok:
            print('error in getting prismatic joint 1 force, error code:',res)
        res,pris2Force=vrep.simxGetJointForce(clientID,prismatic_joint_2,vrep.simx_opmode_buffer)
        print(pris2Force)
        if res!=vrep.simx_return_ok:
            print('error in getting prismatic joint 2 force, error code:',res)
        res,pris3Force=vrep.simxGetJointForce(clientID,prismatic_joint_3,vrep.simx_opmode_buffer)
        print(pris3Force)
        if res!=vrep.simx_return_ok:
            print('error in getting prismatic joint 3 force, error code:',res)
        
        #choosing random values for wheels and prismatic joints velocities (vpr,vpl,vp0r,vp0l,vp1r,vp1l,vpris1,vpris2,vpris3)
        action_buckets=[5,5,5,5,5,5,5,5,5]
        action_max=[0.25918,0.25918,0.25918,0.25918,0.25918,0.25918,0.010472,0.010472,0.010472]
        velInput=numpy.zeros((1,9))
        for i in range(0,6):
            action=random.randrange(1,action_buckets[i]+1)
            range_length=(2*action_max[i])/action_buckets[i]
            a1=action_max[i]-(range_length*(action_buckets[i]-action))
            a0=action_max[i]-(range_length*(action_buckets[i]-(action-1)))
            velInput[0,i]=(a1+a0)/2   #the real action value is the middle point of the bucket that is chosen
        for i in range(6,9):
            action=random.randrange(1,action_buckets[i]+1)
            range_length=(2*action_max[i])/action_buckets[i]
            a1=action_max[i]-(range_length*(action_buckets[i]-action))
            a0=action_max[i]-(range_length*(action_buckets[i]-(action-1)))
            velInput[0,i]=(a1+a0)/2   #the real action value is the middle point of the bucket that is chosen
            
        numExamples=400
        #savingExInputs=numpy.zeros((numExamples,9))
        savingExInputs = numpy.zeros((numExamples, 17))
        savingExOutputs=numpy.zeros((numExamples,2))
        ExCount=0

        IsStopped=0
        ##--- main loop ---
        while vrep.simxGetConnectionId(clientID)!=-1 and ExCount<numExamples-4 and IsStopped==0: #while we are connected to the server..
            IsStopped=StopSimulation(clientID,proximity_sensor_1,proximity_sensor_2,proximity_sensor_3,prismatic_joint_1,prismatic_joint_2,prismatic_joint_3)
            #saving inputs (robot1 right wheel V,r1 left wheel V,r2 right wheel V,r2 left wheel V,
            #                   r1 up-down angle, r1 left-right angle,r2 up-down angle, r2 left-right angle, prismatic target velocity)
            #
            #and outputs (prismatic joint force, possitive (1) or negative (0))
            input_p=NNinput('Pioneer')
            input_p0=NNinput('Pioneer#0')
            input_p1=NNinput('Pioneer#1')
            
            #moving the robots (giving the target velocities and see what the force becomes)
            move(clientID,velInput[0,:])
            
            #getting prismatic joint forces to see if they are possitive or not
            res,pris1Force=vrep.simxGetJointForce(clientID,prismatic_joint_1,vrep.simx_opmode_buffer)
            if res!=vrep.simx_return_ok:
                print('error in getting prismatic joint 1 force, error code:',res)
            res,pris2Force=vrep.simxGetJointForce(clientID,prismatic_joint_2,vrep.simx_opmode_buffer)
            if res!=vrep.simx_return_ok:
                print('error in getting prismatic joint 2 force, error code:',res)
            res,pris3Force=vrep.simxGetJointForce(clientID,prismatic_joint_3,vrep.simx_opmode_buffer)
            if res!=vrep.simx_return_ok:
                print('error in getting prismatic joint 3 force, error code:',res)


           #prismatic 1 between pioneer and pioneer#1
            savingExInputs[ExCount,0]=input_p[1] #rightWheel vel
            savingExInputs[ExCount,1]=input_p[2] #leftWheel vel
            savingExInputs[ExCount,2]=input_p1[1]
            savingExInputs[ExCount,3]=input_p1[2]
            savingExInputs[ExCount,4]=input_p[6]  #leftrope UDangel
            savingExInputs[ExCount,5]=input_p[7]  #rightrope LRangel
            savingExInputs[ExCount,6]=input_p1[6]
            savingExInputs[ExCount,7]=input_p1[7]
            savingExInputs[ExCount, 8] = input_p[0] #tilt
            savingExInputs[ExCount, 9] = input_p1[0] #tilt
            savingExInputs[ExCount, 10] = input_p0[0]  # tilt
            savingExInputs[ExCount,11]=velInput[0,6]    #prismatic vel
            savingExInputs[ExCount, 12] = input_p[9]    #d0
            savingExInputs[ExCount, 13] = input_p[10]   #d1
            savingExInputs[ExCount, 14] = input_p[11]   #d2
            savingExInputs[ExCount, 15] = 3         #diameter
            savingExInputs[ExCount, 16] = 9.331e-02 #weight
            
            savingExOutputs[ExCount,0]=pris1Force
            if pris1Force>0:
                savingExOutputs[ExCount,1]=1

            else:
                savingExOutputs[ExCount,1]=0
                
            ExCount=ExCount+1
            print(savingExOutputs[ExCount-1])
            
            #prismatic 2 between pioneer and pioneer#0
            savingExInputs[ExCount,0]=input_p[1]
            savingExInputs[ExCount,1]=input_p[2]
            savingExInputs[ExCount,2]=input_p0[1]
            savingExInputs[ExCount,3]=input_p0[2]
            savingExInputs[ExCount,4]=input_p[6]
            savingExInputs[ExCount,5]=input_p[7]
            savingExInputs[ExCount,6]=input_p0[6]
            savingExInputs[ExCount,7]=input_p0[7]
            savingExInputs[ExCount, 8] = input_p[0]  # tilt
            savingExInputs[ExCount, 9] = input_p0[0]  # tilt
            savingExInputs[ExCount, 10] = input_p1[0]  # tilt
            savingExInputs[ExCount,11]=velInput[0,7]
            savingExInputs[ExCount, 12] = input_p[9]
            savingExInputs[ExCount, 13] = input_p[10]
            savingExInputs[ExCount, 14] = input_p[11]
            savingExInputs[ExCount, 15] = 3
            savingExInputs[ExCount, 16] = 9.331e-02  # weight

            savingExOutputs[ExCount,0]=pris2Force
            if pris2Force>0:
                savingExOutputs[ExCount,1]=1
            else:
                savingExOutputs[ExCount,1]=0

            ExCount=ExCount+1
            print(savingExOutputs[ExCount - 1])
            
            #prismatic 3 between pioneer#0 and pioneer#1
            savingExInputs[ExCount,0]=input_p0[1]
            savingExInputs[ExCount,1]=input_p0[2]
            savingExInputs[ExCount,2]=input_p1[1]
            savingExInputs[ExCount,3]=input_p1[2]
            savingExInputs[ExCount,4]=input_p0[6]
            savingExInputs[ExCount,5]=input_p0[7]
            savingExInputs[ExCount,6]=input_p1[6]
            savingExInputs[ExCount,7]=input_p1[7]
            savingExInputs[ExCount, 8] = input_p0[0]  # tilt
            savingExInputs[ExCount, 9] = input_p1[0]  # tilt
            savingExInputs[ExCount, 10] = input_p[0]  # tilt
            savingExInputs[ExCount,11]=velInput[0,8]
            savingExInputs[ExCount, 12] = input_p[9]   # d0
            savingExInputs[ExCount, 13] = input_p[10]  # d1
            savingExInputs[ExCount, 14] = input_p[11]  # d2
            savingExInputs[ExCount, 15] = 3
            savingExInputs[ExCount, 16] = 9.331e-02  # weight

            savingExOutputs[ExCount,0]=pris3Force
            if pris3Force>0:
                savingExOutputs[ExCount,1]=1
            else:
                savingExOutputs[ExCount,1]=0

            ExCount=ExCount+1
            print(savingExOutputs[ExCount - 1])
            
            #changing all velocities
            velInput=numpy.zeros((1,9))
            for i in range(0,6):
                action=random.randrange(1,action_buckets[i]+1)
                range_length=(2*action_max[i])/action_buckets[i]
                a1=action_max[i]-(range_length*(action_buckets[i]-action))
                a0=action_max[i]-(range_length*(action_buckets[i]-(action-1)))
                velInput[0,i]=(a1+a0)/2   #the real action value is the middle point of the bucket that is chosen
            for i in range(6,9):
                action=random.randrange(1,action_buckets[i]+1)
                range_length=(2*action_max[i])/action_buckets[i]
                a1=action_max[i]-(range_length*(action_buckets[i]-action))
                a0=action_max[i]-(range_length*(action_buckets[i]-(action-1)))
                velInput[0,i]=(a1+a0)/2   #the real action value is the middle point of the bucket that is chosen

            print(velInput)

            print('count',ExCount)
            

            if ExCount>=numExamples-10:
                #saving generated examples
                name='D:\\internship\\robot final files\\final\\samples\\new\\new4\\inputs'+str(32)+'.npy'
                numpy.save(name,savingExInputs)
                name='D:\\internship\\robot final files\\final\\samples\\new\\new4\\outputs'+str(32)+'.npy'
                numpy.save(name,savingExOutputs)
                name='D:\\internship\\robot final files\\final\\samples\\new\\new4\\ExCount'+str(32)+'.npy'
                numpy.save(name,ExCount)
            
          
        # Stoping the Simulation
        res=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        

        #saving generated examples
        name='D:\\internship\\robot final files\\final\\samples\\new\\new4\\inputs'+str(32)+'.npy'
        numpy.save(name,savingExInputs)
        name='D:\\internship\\robot final files\\final\\samples\\new\\new4\\outputs'+str(32)+'.npy'
        numpy.save(name,savingExOutputs)
        name='D:\\internship\\robot final files\\final\\samples\\new\\new4\\ExCount'+str(32)+'.npy'
        numpy.save(name,ExCount)
    
    
else:
    print ('Failed connecting to remote API server')
    print ('Program ended')

# Now close the connection to V-REP:
vrep.simxFinish(clientID)
