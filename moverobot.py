try:
    import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import time
import numpy
import math
import random
import sys
# sys.path.append(r'C:\Program Files\V-REP3\V-REP_PRO_EDU')
import transforms3d


# math functions
def dotproduct(v1, v2):
    return sum((a * b) for a, b in zip(v1, v2))


def length(v):
    return math.sqrt(dotproduct(v, v))


def angle(v1, v2):
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))


# function: defining the stopping criteria
def StopSimulation(clientID, PS1handle, PS2handle, PS3handle, PJ1handle, PJ2handle, PJ3handle):
    # PS1=proximity_sensor_1 and PJ1=prismatic_joint_1

    isStopped = 0
    # --reading the distance handle data
    resPS1, detectionState1, _, _, _ = vrep.simxReadProximitySensor(clientID, PS1handle, vrep.simx_opmode_buffer)
    if resPS1 != vrep.simx_return_ok:
        print('Cannot read proximity sensor 1')

    resPS2, detectionState2, _, _, _ = vrep.simxReadProximitySensor(clientID, PS2handle, vrep.simx_opmode_buffer)
    if resPS2 != vrep.simx_return_ok:
        print('Cannot read proximity sensor 2')

    resPS3, detectionState3, _, _, _ = vrep.simxReadProximitySensor(clientID, PS3handle, vrep.simx_opmode_buffer)
    #if resPS3 != vrep.simx_return_ok:
    #    print('Cannot read proximity sensor 3')

    # --reading the joint handle forces
    resPJ1, joint_force_1 = vrep.simxGetJointForce(clientID, PJ1handle, vrep.simx_opmode_buffer)
    #if resPJ1 != vrep.simx_return_ok:
    #    print('Cannot read force of prismatic joint 1:', resPJ1)

    resPJ2, joint_force_2 = vrep.simxGetJointForce(clientID, PJ2handle, vrep.simx_opmode_buffer)
    #if resPJ1 != vrep.simx_return_ok:
    #    print('Cannot read force of prismatic joint 2:', resPJ2)

    resPJ3, joint_force_3 = vrep.simxGetJointForce(clientID, PJ3handle, vrep.simx_opmode_buffer)
    #if resPJ1 != vrep.simx_return_ok:
    #    print('Cannot read force of prismatic joint 3:', resPJ3)

    # --stopping the simulation if all the data is being recieved and the robot has flipped or the cables are torn
    if resPS1 == vrep.simx_return_ok and resPS2 == vrep.simx_return_ok and resPS3 == vrep.simx_return_ok and resPJ1 == vrep.simx_return_ok and resPJ2 == vrep.simx_return_ok \
            and resPJ3 == vrep.simx_return_ok:
        if detectionState1 == 0:  # or detectionState2 == 0 or detectionState3 == 0:
            print('Robot flipped. Simulation stopped.')
            # Stoping the Simulation
            res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

            isStopped = 1

        if abs(joint_force_1) > 200 or abs(joint_force_2) > 200 or abs(joint_force_3) > 200:
            print('Cable is torn. Simulation stopped.')
            # Stoping the Simulation
            res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
            isStopped = 1

    return isStopped


# function: moving
def move(ClientID, inputs):
    # inputs=[V_rightWheel,V_leftWheel,V_rightWheel#0,V_leftWheel#0,V_rightWheel#1,V_leftWheel#1,V_prismatic_1,V_prismatic_2,V_prismatic_3]
    # if a value in input array does not change (is the same as before), put 's' instead of a number

    if type(inputs[1]) is not str:
        res = vrep.simxSetJointTargetVelocity(ClientID, P_leftWheel, inputs[1], vrep.simx_opmode_oneshot)
        #res = vrep.simxSetJointTargetVelocity(ClientID, P_leftWheel, -1.04, vrep.simx_opmode_oneshot_wait)
        if res != vrep.simx_return_ok:
            print('error in moving joint P_leftWheel')

    if type(inputs[0]) is not str:
        res = vrep.simxSetJointTargetVelocity(ClientID, P_rightWheel, inputs[0], vrep.simx_opmode_oneshot)
        #res = vrep.simxSetJointTargetVelocity(ClientID, P_rightWheel, -1.04, vrep.simx_opmode_oneshot_wait)
        if res != vrep.simx_return_ok:
            print('error in moving joint P_rightWheel')



# --- main program -------------------------------------------------------------------------------------

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print('Connected to remote API server')

    fileNum = 1004
    # !!---------loading desired scene---------
    # for episodeNum in range(10,EPISODES+4):
    for episodeNum in range(1, 27):
        # make completely sure sim is stopped
        res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        time.sleep(5)
        # close current scene
        res = vrep.simxCloseScene(clientID, vrep.simx_opmode_oneshot_wait)
        if res == vrep.simx_return_ok:
            print('Previous file successfully closed.')
        else:
            print('file did not close with error code:', res)


        fileNum = fileNum + 1
        #fileNum = 2

        print('File number:    ', fileNum)
        name = 'D:\\internship\\robot final files\\final\\samples\\mesh\\test' + str(fileNum) + '.ttt'

        res = vrep.simxLoadScene(clientID, name, 0, vrep.simx_opmode_blocking)
        if res == 0:
            print('file successfully loaded.')

        # vrep.simxSynchronous(clientID,True);

        ##--- getting object handles ---
        # --getting proximity sensor handles
        res, proximity_sensor_1 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor_1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('proximity sensor 1 handle recieved.')
            # initializing the stream that reads sensor
            res, _, _, _, _ = vrep.simxReadProximitySensor(clientID, proximity_sensor_1, vrep.simx_opmode_streaming)
        else:
            print('getting proximity sensor 1: call returned with error code: ', res)

        res, proximity_sensor_2 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor_2', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('proximity sensor 2 handle recieved.')
            # initializing the stream that reads sensor
            res, _, _, _, _ = vrep.simxReadProximitySensor(clientID, proximity_sensor_2, vrep.simx_opmode_streaming)
        else:
            print('getting proximity sensor 2: call returned with error code: ', res)

        res, proximity_sensor_3 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor_3', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('proximity sensor 3 handle recieved.')
            # initializing the stream that reads sensor
            res, _, _, _, _ = vrep.simxReadProximitySensor(clientID, proximity_sensor_3, vrep.simx_opmode_streaming)
        else:
            print('getting proximity sensor 3: call returned with error code: ', res)

        # --getting prismatic joint handles
        res, prismatic_joint_1 = vrep.simxGetObjectHandle(clientID, 'Prismatic_joint_1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('prismatic joint 1 handle recieved.')
            # initializing the stream that reads force of joint
            res, _ = vrep.simxGetJointForce(clientID, prismatic_joint_1, vrep.simx_opmode_streaming)
            res, _ = vrep.simxGetJointPosition(clientID, prismatic_joint_1, vrep.simx_opmode_streaming)
        else:
            print('getting prismatic joint 1: call returned with error code: ', res)

        res, prismatic_joint_2 = vrep.simxGetObjectHandle(clientID, 'Prismatic_joint_2', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('prismatic joint 2 handle recieved.')
            # initializing the stream that reads force of joint
            res, _ = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_streaming)
            res, _ = vrep.simxGetJointPosition(clientID, prismatic_joint_2, vrep.simx_opmode_streaming)
        else:
            print('getting prismatic joint 2: call returned with error code: ', res)

        res, prismatic_joint_3 = vrep.simxGetObjectHandle(clientID, 'Prismatic_joint_3', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('prismatic joint 3 handle recieved.')
            # initializing the stream that reads force of joint
            res, _ = vrep.simxGetJointForce(clientID, prismatic_joint_3, vrep.simx_opmode_streaming)
            res, _ = vrep.simxGetJointPosition(clientID, prismatic_joint_3, vrep.simx_opmode_streaming)
        else:
            print('getting prismatic joint 3: call returned with error code: ', res)

        # --getting wheel handles
        res, P_leftWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer left wheel handle recieved.')
            # initializing the stream that reads position of the wheel
            res, _ = vrep.simxGetJointPosition(clientID, P_leftWheel, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer left wheel: call returned with error code:', res)

        res, P_rightWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer right wheel handle recieved.')
            # initializing the stream that reads position of the wheel
            res, _ = vrep.simxGetJointPosition(clientID, P_rightWheel, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer right wheel: call returned with error code:', res)

        res, P0_leftWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#0 left wheel handle recieved.')
            # initializing the stream that reads position of the wheel
            res, _ = vrep.simxGetJointPosition(clientID, P0_leftWheel, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#0 left wheel: call returned with error code:', res)

        res, P0_rightWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#0 right wheel handle recieved.')
            # initializing the stream that reads position of the wheel
            res, _ = vrep.simxGetJointPosition(clientID, P0_rightWheel, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#0 right wheel: call returned with error code:', res)

        res, P1_leftWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#1 left wheel handle recieved.')
            # initializing the stream that reads position of the wheel
            res, _ = vrep.simxGetJointPosition(clientID, P1_leftWheel, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#1 left wheel: call returned with error code:', res)

        res, P1_rightWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#1 right wheel handle recieved.')
            # initializing the stream that reads position of the wheel
            res, _ = vrep.simxGetJointPosition(clientID, P1_rightWheel, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#1 right wheel: call returned with error code:', res)

        # --getting spherical joint handles
        res, P_leftSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_left', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer left spherical joint handle recieved.')
            # initializing the stream that reads position of the spherical joint
            res, _ = vrep.simxGetJointMatrix(clientID, P_leftSphericalJoint, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer left spherical joint: call returned with error code:', res)

        res, P_rightSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_right', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer right spherical joint handle recieved.')
            # initializing the stream that reads position of the spherical joint
            res, _ = vrep.simxGetJointMatrix(clientID, P_rightSphericalJoint, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer right spherical joint: call returned with error code:', res)

        res, P0_leftSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_left#0', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#0 left spherical joint handle recieved.')
            # initializing the stream that reads position of the spherical joint
            res, _ = vrep.simxGetJointMatrix(clientID, P0_leftSphericalJoint, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#0 left spherical joint: call returned with error code:', res)

        res, P0_rightSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_right#0', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#0 right spherical joint handle recieved.')
            # initializing the stream that reads position of the spherical joint
            res, _ = vrep.simxGetJointMatrix(clientID, P0_rightSphericalJoint, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#0 right spherical joint: call returned with error code:', res)

        res, P1_leftSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_left#1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#1 left spherical joint handle recieved.')
            # initializing the stream that reads position of the spherical joint
            res, _ = vrep.simxGetJointMatrix(clientID, P1_leftSphericalJoint, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#1 left spherical joint: call returned with error code:', res)

        res, P1_rightSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_right#1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#1 right spherical joint handle recieved.')
            # initializing the stream that reads position of the spherical joint
            res, _ = vrep.simxGetJointMatrix(clientID, P1_rightSphericalJoint, vrep.simx_opmode_streaming)
        else:
            print('getting Pioneer#1 right spherical joint: call returned with error code:', res)

        # --getting the revolute upper disk joint handles
        res, P_UpperDiskJoint = vrep.simxGetObjectHandle(clientID, 'Revolute_upperDisk', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('revolute upper disk joint handle recieved.')
            # initializing the stream that reads joint orientation (for tilt)
            res, _ = vrep.simxGetObjectOrientation(clientID, P_UpperDiskJoint, -1, vrep.simx_opmode_streaming)
            # initializing the stream that reads joint position
            res, _ = vrep.simxGetJointPosition(clientID, P_UpperDiskJoint, vrep.simx_opmode_streaming)
        else:
            print('getting revolute upper disk joint: call returned with error code: ', res)

        res, P0_UpperDiskJoint = vrep.simxGetObjectHandle(clientID, 'Revolute_upperDisk#0', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('revolute upper disk joint#0 handle recieved.')
            # initializing the stream that reads joint orientation (for tilt)
            eulerAngle = vrep.simxGetObjectOrientation(clientID, P0_UpperDiskJoint, -1, vrep.simx_opmode_streaming)
            # initializing the stream that reads joint position
            res, _ = vrep.simxGetJointPosition(clientID, P0_UpperDiskJoint, vrep.simx_opmode_streaming)
        else:
            print('getting revolute upper disk joint#0: call returned with error code: ', res)

        res, P1_UpperDiskJoint = vrep.simxGetObjectHandle(clientID, 'Revolute_upperDisk#1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('revolute upper disk joint#1 handle recieved.')
            # initializing the stream that reads joint orientation (for tilt)
            eulerAngle = vrep.simxGetObjectOrientation(clientID, P1_UpperDiskJoint, -1, vrep.simx_opmode_streaming)
            # initializing the stream that reads joint position
            res, _ = vrep.simxGetJointPosition(clientID, P1_UpperDiskJoint, vrep.simx_opmode_streaming)
        else:
            print('getting revolute upper disk joint#1: call returned with error code: ', res)

        # getting the robots' object handles
        res, pioneer = vrep.simxGetObjectHandle(clientID, 'Pioneer', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer recieved.')
        else:
            print('getting pioneer: call returned with error code: ', res)

        res, pioneer0 = vrep.simxGetObjectHandle(clientID, 'Pioneer#0', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#0 recieved.')
        else:
            print('getting pioneer#0: call returned with error code: ', res)

        res, pioneer1 = vrep.simxGetObjectHandle(clientID, 'Pioneer#1', vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print('pioneer#1 recieved.')
        else:
            print('getting pioneer#1: call returned with error code: ', res)


        res, position2 = vrep.simxGetJointPosition(clientID, prismatic_joint_2, vrep.simx_opmode_blocking)
        print('res:', res, 'position:', position2)


        ##--- starting the simulation -------------------------------------------------------------------------------------
        res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        #time.sleep(3)
        if res == vrep.simx_return_ok:
            print('Simulation started')
        else:
            print('Starting Simulation call returned with error code: ', res)
        #res ,retInts ,retFloats ,retStrings ,retBuffer = vrep.simxCallScriptFunction(clientID, 'Prismatic_joint_2',vrep.sim_scripttype_childscript,'set_joint_interval', [],[-10,20], '', bytearray(b'Python is interesting.'),vrep.simx_opmode_blocking)

        # choosing random values for wheels and prismatic joints velocities (vpr,vpl,vp0r,vp0l,vp1r,vp1l,vpris1,vpris2,vpris3)
        action_buckets = [5, 5, 5, 5, 5, 5, 5, 5, 5]
        action_max = [0.25918, 0.25918, 0.25918, 0.25918, 0.25918, 0.25918, 0.010472, 0.010472, 0.010472]
        velInput = numpy.zeros((1, 9))
        for i in range(0, 6):
            action = random.randrange(1, action_buckets[i] + 1)
            range_length = (2 * action_max[i]) / action_buckets[i]
            a1 = action_max[i] - (range_length * (action_buckets[i] - action))
            a0 = action_max[i] - (range_length * (action_buckets[i] - (action - 1)))
            velInput[0, i] = (a1 + a0) / 2  # the real action value is the middle point of the bucket that is chosen
            velInput[0, 0] = 1.04
            velInput[0, 1] = 1.04
            velInput[0, i] = 0
        for i in range(6, 9):
            action = random.randrange(1, action_buckets[i] + 1)
            range_length = (2 * action_max[i]) / action_buckets[i]
            a1 = action_max[i] - (range_length * (action_buckets[i] - action))
            a0 = action_max[i] - (range_length * (action_buckets[i] - (action - 1)))
            velInput[0, i] = (a1 + a0) / 2  # the real action value is the middle point of the bucket that is chosen
            velInput[0, 6] = 0.08
            velInput[0, i] = 0
            # if velInput[0, i]>0:
            #    velInput[0, i]=-1*velInput[0, i]

        numExamples = 10000
        # savingExInputs=numpy.zeros((numExamples,9))
        savingExInputs = numpy.zeros((numExamples, 27))
        savingExOutputs = numpy.zeros((numExamples, 2))
        ExCount = 0

        IsStopped = 0
        ##--- main loop ---
        while vrep.simxGetConnectionId(
                clientID) != -1 and ExCount < numExamples - 4 and IsStopped == 0:  # while we are connected to the server..
            IsStopped = StopSimulation(clientID, proximity_sensor_1, proximity_sensor_2, proximity_sensor_3,
                                       prismatic_joint_1, prismatic_joint_2, prismatic_joint_3)

            res, position1 = vrep.simxGetObjectPosition(clientID, pioneer1, -1, vrep.simx_opmode_blocking)

            print(' ')

            # getting prismatic joint forces to see if they are possitive or not
            res, pris2Force = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_buffer)
            print('pris2Force',pris2Force)
            #res, pris3Force = vrep.simxGetJointForce(clientID, prismatic_joint_3, vrep.simx_opmode_buffer)

            res, pris2Pos = vrep.simxGetJointPosition(clientID, prismatic_joint_2, vrep.simx_opmode_blocking)

            velInput = numpy.zeros((1, 9))
            for i in range(0, 6):
                action = random.randrange(1, action_buckets[i] + 1)
                range_length = (2 * action_max[i]) / action_buckets[i]
                a1 = action_max[i] - (range_length * (action_buckets[i] - action))
                a0 = action_max[i] - (range_length * (action_buckets[i] - (action - 1)))
                velInput[0, i] = (a1 + a0) / 2  # the real action value is the middle point of the bucket that is chosen
                velInput[0, i] = velInput[0, i]*10 
                # velInput[0, 1] = 1.04

            if pris2Force==0:
                move(clientID, velInput[0, :])
                res, pris2Pos = vrep.simxGetJointPosition(clientID, prismatic_joint_2, vrep.simx_opmode_blocking)
                print('pos2:', pris2Pos)
                #res, pris3Pos = vrep.simxGetJointPosition(clientID, prismatic_joint_3, vrep.simx_opmode_blocking)
                #print('pos3:', pris3Pos)

            if pris2Force!=0:
                print('in force mode')
                print('force2:',pris2Force)
                #print('force3:', pris3Force)
                if pris2Pos==0:
                    print('not moving')

                #to change rope range and max
                joint_min, joint_range=-2.00,4.00
                #res ,retInts ,retFloats ,retStrings ,retBuffer = vrep.simxCallScriptFunction(clientID, 'Prismatic_joint_2',vrep.sim_scripttype_childscript,'set_joint_interval', [],[joint_min,joint_range], '', bytearray(b'Python is interesting.'),vrep.simx_opmode_blocking)

                move(clientID, velInput[0, :])
                for n in range(1,2):
                    time.sleep(5)
                    res, pris2Force = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_buffer)
                    res, pris2Pos = vrep.simxGetJointPosition(clientID, prismatic_joint_2, vrep.simx_opmode_blocking)
                    #print('res:',res)
                    if pris2Force==0:
                        print('no force')
                        print('time: ',n*0.002)
                        print('pos: ',pris2Pos)
                    else:
                        print('force:',pris2Force)
                        print('pose:',pris2Pos)
            res, pris2Pos = vrep.simxGetJointPosition(clientID, prismatic_joint_2, vrep.simx_opmode_blocking)
            #res, pris3Pos = vrep.simxGetJointPosition(clientID, prismatic_joint_3, vrep.simx_opmode_blocking)
            print('pos2:',pris2Pos)
            #print('pos3:', pris3Pos)
            print(' ')


            print('count', ExCount)
            #time.sleep(1)

        # Stoping the Simulation
        res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)



else:
    print('Failed connecting to remote API server')
    print('Program ended')

# Now close the connection to V-REP:
vrep.simxFinish(clientID)
