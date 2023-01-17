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
from time import sleep
import numpy
import math
import random
from collections import deque
import keras
from keras.models import load_model
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Dropout
from keras import regularizers
from keras.optimizers import Adam
from keras.callbacks import TensorBoard
import matplotlib.pyplot as plt
from sklearn import preprocessing
import tensorflow as tf
# (!!temp!!)
import sys

sys.path.append(r'C:\Users\Sharareh.DELL_PC\Anaconda3\Lib\site-packages')
sys.path.append(r'C:\Program Files\V-REP3\V-REP_PRO_EDU')
import transforms3d


##---------------------------------------------------------------**** Defining Functions ****--------------------------------------------------------------------------
##---------------------------------------------------------------------------------------------------------------------------------------------------------------------


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

    time = 0

    # --reading the distance handle data
    resPS1, detectionState1, _, _, _ = vrep.simxReadProximitySensor(clientID, PS1handle, vrep.simx_opmode_buffer)
    if resPS1 != vrep.simx_return_ok:
        print('Cannot read proximity sensor 1')

    resPS2, detectionState2, _, _, _ = vrep.simxReadProximitySensor(clientID, PS2handle, vrep.simx_opmode_buffer)
    if resPS2 != vrep.simx_return_ok:
        print('Cannot read proximity sensor 2')

    resPS3, detectionState3, _, _, _ = vrep.simxReadProximitySensor(clientID, PS3handle, vrep.simx_opmode_buffer)
    if resPS3 != vrep.simx_return_ok:
        print('Cannot read proximity sensor 3')

    # --reading the joint handle forces
    resPJ1, joint_force_1 = vrep.simxGetJointForce(clientID, PJ1handle, vrep.simx_opmode_buffer)
    if resPJ1 != vrep.simx_return_ok:
        print('Cannot read force of prismatic joint 1:', resPJ1)

    resPJ2, joint_force_2 = vrep.simxGetJointForce(clientID, PJ2handle, vrep.simx_opmode_buffer)
    if resPJ1 != vrep.simx_return_ok:
        print('Cannot read force of prismatic joint 2:', resPJ2)

    resPJ3, joint_force_3 = vrep.simxGetJointForce(clientID, PJ3handle, vrep.simx_opmode_buffer)
    if resPJ1 != vrep.simx_return_ok:
        print('Cannot read force of prismatic joint 3:', resPJ3)

    # --stopping the simulation if all the data is being recieved and the robot has flipped or the cables are torn
    if resPS1 == vrep.simx_return_ok and resPS2 == vrep.simx_return_ok and resPS3 == vrep.simx_return_ok and resPJ1 == vrep.simx_return_ok and resPJ2 == vrep.simx_return_ok \
            and resPJ3 == vrep.simx_return_ok:
        is_stopped = 0
        if detectionState1 == 0:
            is_stopped = 1
            print('Robot flipped. Simulation stopped.')
            time = vrep.simxGetLastCmdTime(clientID)
            print('time: ', time / 1000)
            print('det1:', detectionState1, '    ', 'det2:', detectionState2, '     ', 'det3:', detectionState3)
            # Stoping the Simulation
            res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            # Now close the connection to V-REP:
            # vrep.simxFinish(clientID)

        if detectionState2 == 0:
            is_stopped = 2
            print('Robot flipped. Simulation stopped.')
            time = vrep.simxGetLastCmdTime(clientID)
            print('time: ', time / 1000)
            print('det1:', detectionState1, '    ', 'det2:', detectionState2, '     ', 'det3:', detectionState3)
            # Stoping the Simulation
            res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            # Now close the connection to V-REP:
            # vrep.simxFinish(clientID)

        if detectionState3 == 0:
            is_stopped = 3
            print('Robot flipped. Simulation stopped.')
            time = vrep.simxGetLastCmdTime(clientID)
            print('time: ', time / 1000)
            print('det1:', detectionState1, '    ', 'det2:', detectionState2, '     ', 'det3:', detectionState3)
            # Stoping the Simulation
            res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            # Now close the connection to V-REP:
            # vrep.simxFinish(clientID)

        if abs(joint_force_1) > 200 or abs(joint_force_2) > 200 or abs(joint_force_3) > 200:
            is_stopped = 4
            print('Cable is torn. Simulation stopped.')
            time = vrep.simxGetLastCmdTime(clientID)
            print('time: ', time / 1000)
            # Stoping the Simulation
            res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
            # Now close the connection to V-REP:
            # vrep.simxFinish(clientID)
        return [is_stopped, time]


# function: check rope like behaviour
def checkRopeBehave(inputs):
    # instructions are the inputs given to move funciton
    # function returns an array of three, [pris1,pris2,pris3] in which 0 means
    # not-rope-like behaviour and 1 means rope-like-behaviour (which is OK)
    # if no change in velocity is given for certain joint, returns 2 for that joint

    input_p = NNinput('Pioneer')
    input_p0 = NNinput('Pioneer#0')
    input_p1 = NNinput('Pioneer#1')

    # for pris 1 (inputs[6])
    check_rope_NN_input = numpy.zeros([15, 1])
    if inputs[6] is not str:
        check_rope_NN_input[0] = input_p[1]
        check_rope_NN_input[1] = input_p[2]
        check_rope_NN_input[2] = input_p1[1]
        check_rope_NN_input[3] = input_p1[2]
        check_rope_NN_input[4] = input_p[6]
        check_rope_NN_input[5] = input_p[7]
        check_rope_NN_input[6] = input_p1[6]
        check_rope_NN_input[7] = input_p1[7]
        check_rope_NN_input[8] = input_p[0]  # tilt
        check_rope_NN_input[9] = input_p1[0]  # tilt
        check_rope_NN_input[10] = input_p0[0]  # tilt
        check_rope_NN_input[11] = inputs[6]
        check_rope_NN_input[12] = input_p[9]
        check_rope_NN_input[13] = input_p[10]
        check_rope_NN_input[14] = input_p[11]

        check_rope_NN_input = numpy.reshape(check_rope_NN_input, (1, 15))
        check_rope_NN_input = preprocessing.normalize(check_rope_NN_input)
        # pris1=ropeModel.predict(check_rope_NN_input)[0][0]
        preddd = ropeModel.predict(check_rope_NN_input)
        pris1 = numpy.argmax(preddd) - 1
        # print('NN result:',pris1)
        predict1 = pris1
        if pris1 == -1:
            pris1 = 1
        else:
            pris1 = 0  # force mode
    else:
        pris1 = 2

    # for pris 2 (inputs[7])
    check_rope_NN_input = numpy.zeros([15, 1])
    if inputs[7] is not str:
        check_rope_NN_input[0] = input_p[1]
        check_rope_NN_input[1] = input_p[2]
        check_rope_NN_input[2] = input_p0[1]
        check_rope_NN_input[3] = input_p0[2]
        check_rope_NN_input[4] = input_p[6]
        check_rope_NN_input[5] = input_p[7]
        check_rope_NN_input[6] = input_p0[6]
        check_rope_NN_input[7] = input_p0[7]
        check_rope_NN_input[8] = input_p[0]  # tilt
        check_rope_NN_input[9] = input_p0[0]  # tilt
        check_rope_NN_input[10] = input_p1[0]  # tilt
        check_rope_NN_input[11] = inputs[7]
        check_rope_NN_input[12] = input_p[9]
        check_rope_NN_input[13] = input_p[10]
        check_rope_NN_input[14] = input_p[11]

        check_rope_NN_input = numpy.reshape(check_rope_NN_input, (1, 15))
        # pris2=ropeModel.predict(check_rope_NN_input)[0][0]
        check_rope_NN_input = preprocessing.normalize(check_rope_NN_input)
        pris2 = numpy.argmax(ropeModel.predict(check_rope_NN_input)) - 1
        # print('NN result:',pris2)
        predict2 = pris2
        if pris2 == -1:
            pris2 = 1
        else:
            pris2 = 0  # force mode
    else:
        pris2 = 2

    # for pris 3 (inputs[8])
    check_rope_NN_input = numpy.zeros([15, 1])
    if inputs[8] is not str:
        check_rope_NN_input[0] = input_p0[1]
        check_rope_NN_input[1] = input_p0[2]
        check_rope_NN_input[2] = input_p1[1]
        check_rope_NN_input[3] = input_p1[2]
        check_rope_NN_input[4] = input_p0[6]
        check_rope_NN_input[5] = input_p0[7]
        check_rope_NN_input[6] = input_p1[6]
        check_rope_NN_input[7] = input_p1[7]
        check_rope_NN_input[8] = input_p0[0]  # tilt
        check_rope_NN_input[9] = input_p1[0]  # tilt
        check_rope_NN_input[10] = input_p[0]  # tilt
        check_rope_NN_input[11] = inputs[8]
        check_rope_NN_input[12] = input_p[9]
        check_rope_NN_input[13] = input_p[10]
        check_rope_NN_input[14] = input_p[11]

        check_rope_NN_input = numpy.reshape(check_rope_NN_input, (1, 15))
        # pris3=ropeModel.predict(check_rope_NN_input)[0][0]
        check_rope_NN_input = preprocessing.normalize(check_rope_NN_input)
        pris3 = numpy.argmax(ropeModel.predict(check_rope_NN_input)) - 1
        # print('NN result:',pris3)
        predict3 = pris3
        if pris3 == -1:
            pris3 = 1
        else:
            pris3 = 0  # force mode
    else:
        pris3 = 2

    return [pris1, pris2, pris3]


# function: moving
def move(clientID, inputs):
    # inputs=[V_rightWheel,V_leftWheel,V_rightWheel#0,V_leftWheel#0,V_rightWheel#1,V_leftWheel#1,V_prismatic_1,
    # V_prismatic_2,V_prismatic_3]
    # if a value in input array does not change (is the same as before), put 's' instead of a number

    # check velocities' validity with checkRopeBehave
    ropeBehave = checkRopeBehave(inputs)

    if type(inputs[6]) is not str:
        if ropeBehave[0] == 0:
            print('positive1')
            # set the joint force to zero
            res = vrep.simxSetJointForce(clientID, prismatic_joint_1, 0, vrep.simx_opmode_oneshot)
            # res = vrep.simxSetJointTargetVelocity(clientID, prismatic_joint_1, 0, vrep.simx_opmode_oneshot_wait)
            if res != vrep.simx_return_ok:
                print('prismatic joint 1 mode was not changed, error code:', res)
            # assign new velocity for joint
        else:
            print('negative1')

            # set the joint force to 200
            res = vrep.simxSetJointForce(clientID, prismatic_joint_1, 200, vrep.simx_opmode_oneshot)
            # res = vrep.simxSetJointForce(clientID, prismatic_joint_1, 0, vrep.simx_opmode_oneshot)
            if res != vrep.simx_return_ok:
                print('prismatic joint 1 mode was not changed, error code:', res)
            # change joint velocity
            res = vrep.simxSetJointTargetVelocity(clientID, prismatic_joint_1, inputs[6], vrep.simx_opmode_oneshot_wait)
            # res = vrep.simxSetJointTargetVelocity(clientID, prismatic_joint_1, 10, vrep.simx_opmode_oneshot_wait)
            if res != vrep.simx_return_ok:
                print('error in setting rope 1 length')
        # get joint force and check to see if it is positive (ie: pushing) which is unacceptable for ropes
        res, prisForce1 = vrep.simxGetJointForce(clientID, prismatic_joint_1, vrep.simx_opmode_buffer)
        if res != vrep.simx_return_ok:
            print('error in getting prismatic joint 1 force, error code:', res)

        print('           prisforce1:', prisForce1)

    if type(inputs[7]) is not str:
        if ropeBehave[1] == 0:
            print('positive2')
            # set the joint force to zero
            res = vrep.simxSetJointForce(clientID, prismatic_joint_2, 0, vrep.simx_opmode_oneshot)
            if res != vrep.simx_return_ok:
                print('prismatic joint 2 mode was not changed, error code:', res)
        # assign new velocity for joint
        else:
            print('negative2')

            # set the joint force to 200
            res = vrep.simxSetJointForce(clientID, prismatic_joint_2, 200, vrep.simx_opmode_oneshot)
            if res != vrep.simx_return_ok:
                print('prismatic joint 2 mode was not changed, error code:', res)
            # change the joint velocity
            res = vrep.simxSetJointTargetVelocity(clientID, prismatic_joint_2, inputs[7], vrep.simx_opmode_oneshot_wait)
            if res != vrep.simx_return_ok:
                print('error in setting rope 2 length')
        # get joint force and check to see if it is positive (ie: pushing) which is unacceptable for ropes
        res, prisForce2 = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_buffer)
        if res != vrep.simx_return_ok:
            print('error in getting prismatic joint 2 force, error code:', res)

        print('           prisforce2:', prisForce2)

    if type(inputs[8]) is not str:
        if ropeBehave[2] == 0:
            print('positive3')
            # set the joint force to zero
            res = vrep.simxSetJointForce(clientID, prismatic_joint_3, 0, vrep.simx_opmode_oneshot)

            if res != vrep.simx_return_ok:
                print('prismatic joint 3 mode was not changed, error code:', res)
        else:
            print('negative3')

            # set the joint force to 200
            res = vrep.simxSetJointForce(clientID, prismatic_joint_3, 200, vrep.simx_opmode_oneshot)
            if res != vrep.simx_return_ok:
                print('prismatic joint 1 mode was not changed, error code:', res)
            # assign new velocity for joint
            res = vrep.simxSetJointTargetVelocity(clientID, prismatic_joint_3, inputs[8], vrep.simx_opmode_oneshot_wait)
            if res != vrep.simx_return_ok:
                print('error in setting rope 3 length')
        # get joint force and check to see if it is positive (ie: pushing) which is unacceptable for ropes
        res, prisForce3 = vrep.simxGetJointForce(clientID, prismatic_joint_3, vrep.simx_opmode_buffer)
        if res != vrep.simx_return_ok:
            print('error in getting prismatic joint 3 force, error code:', res)

        print('           prisforce3:', prisForce3)

    if type(inputs[5]) is not str:
        res = vrep.simxSetJointTargetVelocity(clientID, P1_leftWheel, inputs[5], vrep.simx_opmode_streaming)
        if res != vrep.simx_return_ok:
            print('error in moving joint P1_leftWheel')

    if type(inputs[4]) is not str:
        res = vrep.simxSetJointTargetVelocity(clientID, P1_rightWheel, inputs[4], vrep.simx_opmode_streaming)
        if res != vrep.simx_return_ok:
            print('error in moving joint P1_rightWheel')

    if type(inputs[3]) is not str:
        res = vrep.simxSetJointTargetVelocity(clientID, P0_leftWheel, inputs[3], vrep.simx_opmode_streaming)
        if res != vrep.simx_return_ok:
            print('error in moving joint P0_leftWheel')

    if type(inputs[2]) is not str:
        res = vrep.simxSetJointTargetVelocity(clientID, P0_rightWheel, inputs[2], vrep.simx_opmode_streaming)
        if res != vrep.simx_return_ok:
            print('error in moving joint P0_rightWheel')

    if type(inputs[1]) is not str:
        res = vrep.simxSetJointTargetVelocity(clientID, P_leftWheel, inputs[1], vrep.simx_opmode_streaming)
        if res != vrep.simx_return_ok:
            print('error in moving joint P_leftWheel')

    if type(inputs[0]) is not str:
        res = vrep.simxSetJointTargetVelocity(clientID, P_rightWheel, inputs[0], vrep.simx_opmode_streaming)
        if res != vrep.simx_return_ok:
            print('error in moving joint P_leftWheel')

    return ropeBehave, prisForce1, prisForce2, prisForce3


# function: find an object's z axis in the main coordinations
def zAxis(Euler):
    quant = transforms3d.euler.euler2quat(Euler[0], Euler[1], Euler[2], axes='sxyz')
    vector = transforms3d.quaternions.rotate_vector([0, 0, 1], quant)
    return vector


# function: find an object's y axis in the main coordinations
def yAxis(Euler):
    quant = transforms3d.euler.euler2quat(Euler[0], Euler[1], Euler[2], axes='sxyz')
    vector = transforms3d.quaternions.rotate_vector([0, 1, 0], quant)
    return vector


# function: find an object's x axis in the main coordinations
def xAxis(Euler):
    quant = transforms3d.euler.euler2quat(Euler[0], Euler[1], Euler[2], axes='sxyz')
    vector = transforms3d.quaternions.rotate_vector([1, 0, 0], quant)
    return vector


# function: get all the inputs for one robot
def NNinput(robotName):
    # inputs: 1)tilt=90-gama  2)right wheel velocity  3)left wheel velocity
    #        4)right rope tension  5)left rope tension  6)right rope up-down angle
    #        7)left rope up-down angle  8)right rope left-right angle
    #        9)left rope left-right angle

    # robotName: 'Pioneer' or 'Pioneer#0' or 'Pioneer#1'

    if robotName == 'Pioneer':
        # getting the wheel velocities
        res, rightWheel_V = vrep.simxGetObjectFloatParameter(clientID, P_rightWheel, 2012, vrep.simx_opmode_blocking)
        res, leftWheel_V = vrep.simxGetObjectFloatParameter(clientID, P_leftWheel, 2012, vrep.simx_opmode_blocking)

        # getting the tilt angle
        res, euler = vrep.simxGetObjectOrientation(clientID, P_UpperDiskJoint, -1, vrep.simx_opmode_blocking)
        z = zAxis(euler)
        tilt = (numpy.pi / 2) - angle(z, [0, 0, 1])

        # getting rope tensions
        res, leftRopeTension = vrep.simxGetJointForce(clientID, prismatic_joint_1, vrep.simx_opmode_buffer)
        res, rightRopeTension = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_buffer)

        # getting ropes' up-down angle (90-angle with pioneer coordinates' z axis)
        # and left-right angles (angle with pioneer coordinates' x axis)
        res, euler = vrep.simxGetObjectOrientation(clientID, prismatic_joint_1, pioneer,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
        z = zAxis(euler)
        leftRopeUDAngle = (numpy.pi / 2) - angle(z, [0, 0, 1])
        leftRopeLRAngle = angle([z[0], z[1]], [1, 0])

        res, euler = vrep.simxGetObjectOrientation(clientID, prismatic_joint_2, pioneer,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
        z = zAxis(euler)
        rightRopeUDAngle = (numpy.pi / 2) - angle(z, [0, 0, 1])
        rightRopeLRAngle = angle([z[0], z[1]], [1, 0])

        res, position = vrep.simxGetObjectPosition(clientID, pioneer, -1, vrep.simx_opmode_blocking)
        res, position0 = vrep.simxGetObjectOrientation(clientID, pioneer0, -1, vrep.simx_opmode_blocking)
        res, position1 = vrep.simxGetObjectOrientation(clientID, pioneer1, -1, vrep.simx_opmode_blocking)
        x, y, z = position
        x0, y0, z0 = position0
        x1, y1, z1 = position1

        d0 = pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2)
        d1 = pow(x - x1, 2) + pow(y - y1, 2) + pow(z - z1, 2)
        d2 = pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2)

    if robotName == 'Pioneer#0':
        # getting the wheel velocities
        res, rightWheel_V = vrep.simxGetObjectFloatParameter(clientID, P0_rightWheel, 2012, vrep.simx_opmode_blocking)
        res, leftWheel_V = vrep.simxGetObjectFloatParameter(clientID, P0_leftWheel, 2012, vrep.simx_opmode_blocking)

        # getting the tilt angle
        res, euler = vrep.simxGetObjectOrientation(clientID, P0_UpperDiskJoint, -1, vrep.simx_opmode_blocking)
        z = zAxis(euler)
        tilt = (numpy.pi / 2) - angle(z, [0, 0, 1])

        # getting rope tensions
        res, leftRopeTension = vrep.simxGetJointForce(clientID, prismatic_joint_3, vrep.simx_opmode_buffer)
        res, rightRopeTension = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_buffer)

        # getting ropes' up-down angle (90-angle with pioneer coordinates' z axis)
        # and left-right angles (angle with pioneer coordinates' x axis)
        res, euler = vrep.simxGetObjectOrientation(clientID, prismatic_joint_3, pioneer0,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
        z = zAxis(euler)
        leftRopeUDAngle = (numpy.pi / 2) - angle(z, [0, 0, 1])
        leftRopeLRAngle = angle([z[0], z[1]], [1, 0])

        res, euler = vrep.simxGetObjectOrientation(clientID, prismatic_joint_2, pioneer0,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
        z = zAxis(euler)
        rightRopeUDAngle = (numpy.pi / 2) - angle(z, [0, 0, 1])
        rightRopeLRAngle = angle([z[0], z[1]], [1, 0])

        res, position = vrep.simxGetObjectPosition(clientID, pioneer, -1, vrep.simx_opmode_blocking)
        res, position0 = vrep.simxGetObjectOrientation(clientID, pioneer0, -1, vrep.simx_opmode_blocking)
        res, position1 = vrep.simxGetObjectOrientation(clientID, pioneer1, -1, vrep.simx_opmode_blocking)
        x, y, z = position
        x0, y0, z0 = position0
        x1, y1, z1 = position1

        d0 = pow(x0 - x, 2) + pow(y0 - y, 2) + pow(z0 - z, 2)
        d1 = pow(x - x1, 2) + pow(y - y1, 2) + pow(z - z1, 2)
        d2 = pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2)

    if robotName == 'Pioneer#1':
        # getting the wheel velocities
        res, rightWheel_V = vrep.simxGetObjectFloatParameter(clientID, P1_rightWheel, 2012, vrep.simx_opmode_blocking)
        res, leftWheel_V = vrep.simxGetObjectFloatParameter(clientID, P1_leftWheel, 2012, vrep.simx_opmode_blocking)

        # getting the tilt angle
        res, euler = vrep.simxGetObjectOrientation(clientID, P1_UpperDiskJoint, -1, vrep.simx_opmode_blocking)
        z = zAxis(euler)
        tilt = (numpy.pi / 2) - angle(z, [0, 0, 1])

        # getting rope tensions
        res, leftRopeTension = vrep.simxGetJointForce(clientID, prismatic_joint_3, vrep.simx_opmode_buffer)
        res, rightRopeTension = vrep.simxGetJointForce(clientID, prismatic_joint_1, vrep.simx_opmode_buffer)

        # getting ropes' up-down angle (90-angle with pioneer coordinates' z axis)
        # and left-right angles (angle with pioneer coordinates' x axis)
        res, euler = vrep.simxGetObjectOrientation(clientID, prismatic_joint_3, pioneer1,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
        z = zAxis(euler)
        leftRopeUDAngle = (numpy.pi / 2) - angle(z, [0, 0, 1])
        leftRopeLRAngle = angle([z[0], z[1]], [1, 0])

        res, euler = vrep.simxGetObjectOrientation(clientID, prismatic_joint_1, pioneer1,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
        z = zAxis(euler)
        rightRopeUDAngle = (numpy.pi / 2) - angle(z, [0, 0, 1])
        rightRopeLRAngle = angle([z[0], z[1]], [1, 0])

        res, position = vrep.simxGetObjectPosition(clientID, pioneer, -1, vrep.simx_opmode_blocking)
        res, position0 = vrep.simxGetObjectOrientation(clientID, pioneer0, -1, vrep.simx_opmode_blocking)
        res, position1 = vrep.simxGetObjectOrientation(clientID, pioneer1, -1, vrep.simx_opmode_blocking)
        x, y, z = position
        x0, y0, z0 = position0
        x1, y1, z1 = position1

        d0 = pow(x0 - x, 2) + pow(y0 - y, 2) + pow(z0 - z, 2)
        d1 = pow(x - x1, 2) + pow(y - y1, 2) + pow(z - z1, 2)
        d2 = pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2)

    return [tilt, rightWheel_V, leftWheel_V, rightRopeTension, leftRopeTension, rightRopeUDAngle, leftRopeUDAngle,
            rightRopeLRAngle, leftRopeLRAngle, d0, d1, d2]


# function: Calculating stability parameter
def stabilityParam(clientID, pioneer, pioneer0, pioneer1, prismatic_joint_1, prismatic_joint_2, prismatic_joint_3,
                   P_leftSphericalJoint, P_rightSphericalJoint, P0_leftSphericalJoint, P0_rightSphericalJoint,
                   P1_leftSphericalJoint, P1_rightSphericalJoint, robotName, mass, mu):
    # robotName: 'Pioneer' , 'Pioneer#0' , 'Pioneer#1' -- mass:robot mass

    if robotName == 'Pioneer':
        # rope tension vectors
        joint_l = prismatic_joint_1
        joint_r = prismatic_joint_2
        robot = pioneer
        leftSphericalJoint=P_leftSphericalJoint
        rightSphericalJoint = P_rightSphericalJoint
    if robotName == 'Pioneer#0':
        joint_l = prismatic_joint_3
        joint_r = prismatic_joint_2
        robot = pioneer0
        leftSphericalJoint = P0_leftSphericalJoint
        rightSphericalJoint = P0_rightSphericalJoint
    if robotName == 'Pioneer#1':
        joint_l = prismatic_joint_3
        joint_r = prismatic_joint_1
        robot = pioneer1
        leftSphericalJoint = P1_leftSphericalJoint
        rightSphericalJoint = P1_rightSphericalJoint

    res, leftRopeTension = vrep.simxGetJointForce(clientID, joint_l, vrep.simx_opmode_buffer)
    res, rightRopeTension = vrep.simxGetJointForce(clientID, joint_r, vrep.simx_opmode_buffer)
    res, euler = vrep.simxGetObjectOrientation(clientID, joint_l, pioneer,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
    tLeft = zAxis(euler)
    res, euler = vrep.simxGetObjectOrientation(clientID, joint_r, pioneer,
                                                   vrep.simx_opmode_blocking)  # euler relative to pioneer coordinate system
    tRight = zAxis(euler)

    # getting velocity and robot coordinate system
    res, linearVelocity, _ = vrep.simxGetObjectVelocity(clientID, robot, vrep.simx_opmode_buffer)
    res, euler = vrep.simxGetObjectOrientation(clientID, root, -1, vrep.simx_opmode_blocking)
    z = zAxis(euler)
    y = yAxis(euler)
    x = xAxis(euler)

    # calculating moment of inertia around robot's x and y axis caused by rope tensions ([I think] friction does
    # not cause moment, and center of mass=center of V)
    res, leftTPosition = vrep.simxGetObjectPosition(clientID, leftSphericalJoint, robot,
                                                        vrep.simx_opmode_blocking)
    res, rightTPosition = vrep.simxGetObjectPosition(clientID, rightSphericalJoint, robot,
                                                         vrep.simx_opmode_blocking)

    

    # direction of movement and friction
    Vx = numpy.dot(linearVelocity, x)
    Vy = numpy.dot(linearVelocity, y)
    Vz = numpy.dot(linearVelocity, z)
    Vmag = numpy.linalg.norm([Vx, Vy, Vz])  # magnitude of v
    Vdir = [Vx / Vmag, Vy / Vmag, Vz / Vmag]  # direction (normalized) of v and friction (in robot coordinate system)
    # direction of gravity in robot coordinate system
    Gx = numpy.dot([0, 0, -1], x)
    Gy = numpy.dot([0, 0, -1], y)
    Gz = numpy.dot([0, 0, -1], z)

    # landas (friction direction vectors)
    landa1 = -Vdir[0]
    landa2 = -Vdir[1]

    # normal force
    N = -((mass * 9.81) * Gz + (tRight[2] * rightRopeTension) + (tLeft[2] * leftRopeTension))

    # Fy
    Fy = (mass * 9.81 * Gy) + (tLeft[1] * leftRopeTension) + (tRight[1] * rightRopeTension)

    # Fx
    Fx = (mass * 9.81 * Gx) + tLeft[0] * leftRopeTension + tRight[0] * rightRopeTension

    """if robotName=='Pioneer':
        print('N:',N,'  FY:',Fy,'   FX:',Fx)"""

    # stability parameters
    SPy = (min(landa2 * mu * N - Fy, landa2 * mu * N + Fy)) / (2 * landa2 * mu * N)
    SPz = (min(landa1 * mu * N - Fx, landa1 * mu * N + Fx)) / (2 * landa1 * mu * N)

    # moments of Ts around center of volume
    Mleft = numpy.cross(leftTPosition, tLeft)
    Mright = numpy.cross(rightTPosition, tRight)
    M = Mleft + Mright
    MPunishment = numpy.linalg.norm(M)  # magnitude of vector M

    zeroTPunishment = 0
    if leftRopeTension == 0:
        zeroTPunishment = 0.1 * (1 / N) + zeroTPunishment
        # zeroTPunishment = (1 / N) + zeroTPunishment
    if rightRopeTension == 0:
        zeroTPunishment = 0.1 * (1 / N) + zeroTPunishment
        # zeroTPunishment = (1 / N) + zeroTPunishment

    """if robotName =='Pioneer':
        print('MP:',MPunishment,'            zeroTP:',zeroTPunishment)
        print('reward: ',N - MPunishment - zeroTPunishment)"""

    return N - MPunishment - zeroTPunishment



def check_loss(state, next_state, action_num, reward, agent):
    q_sa = agent.predict(state)[0][int(action_num)]
    q_nextsa = numpy.amax(agent.predict(next_state)[0])
    loss = pow(q_sa - (reward + agent.gamma * q_nextsa), 2)
    return loss



if __name__ == "__main__":
    filename = "test11gamma.txt"
    f = open(filename, "a+")
    
    rope = []
    positive = 0
    negitive = 0
    
    all_loss = numpy.zeros(2000)
    all_loss0 = numpy.zeros(2000)
    all_loss1 = numpy.zeros(2000)
    loss_num = 0
    
    EPISODES = 15  # how many times to start from a new starting point
    start_points = 17  # how many starting point files we have
    scene_changed = 1
    flipped = 0
    iteration = 0
    max_iteration = 8
    full_steps = 0
    state_size = 9  # inputs (simular to NNinput): 1)tilt=90-gama  2)right wheel velocity  3)left wheel velocity
    #        4)right rope tension  5)left rope tension  6)right rope up-down angle
    #        7)left rope up-down angle  8)right rope left-right angle
    #        9)left rope left-right angle
    action_size = 3  # how many actions, we have 3: [rightWheel,leftWheel,prismatic]
    agent = DQNAgent.DQNAgent(state_size, action_size)
    
    reward = 0
    reward0 = 0
    reward1 = 0
    ## Load the rope behaviour prediction neural network named: 'ropeModel'
    ropeModel = load_model('D:/internship/robot final files/final/ropeModelnew7.h5')
    
    end_time = []
    end_step = []
    pauseNum = 0
    
    NNErrorLog = numpy.zeros([1, 6])
    
    print('Program started')
    vrep.simxFinish(-1)  # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
    if clientID != -1:
        print('Connected to remote API server')
    
        # !!---------loading desired scene---------
        for episodeNum in range(EPISODES * max_iteration):
            isStopped = 0
            flipped = 0
            # close current scene
            res = vrep.simxCloseScene(clientID, vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                print('Previous file successfully closed.')
    
            if iteration == max_iteration:
                scene_changed = 1
                iteration = 0
    
            # name of file
            if scene_changed == 1:
                fileNum = random.randint(1, start_points)
                # fileNum = random.randint(11, 11)
                # scene_changed=1
    
            iteration += 1
            print('File number:    ', fileNum)
            name = 'D:/internship/robot final files/final/samples/1/test' + str(fileNum) + '.ttt'
            res = vrep.simxLoadScene(clientID, name, 0, vrep.simx_opmode_blocking)
            print(res)
            if res == 0:
                print('file successfully loaded.')
    
            # vrep.simxSynchronous(clientID,True);
    
            ##--- getting object handles ---
            # --getting proximity sensor handles
            res, proximity_sensor_1 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor_1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # initializing the stream that reads sensor
                res, _, _, _, _ = vrep.simxReadProximitySensor(clientID, proximity_sensor_1, vrep.simx_opmode_streaming)
            else:
                print('getting proximity sensor 1: call returned with error code: ', res)
    
            res, proximity_sensor_2 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor_2', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # initializing the stream that reads sensor
                res, _, _, _, _ = vrep.simxReadProximitySensor(clientID, proximity_sensor_2, vrep.simx_opmode_streaming)
            else:
                print('getting proximity sensor 2: call returned with error code: ', res)
    
            res, proximity_sensor_3 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor_3', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # initializing the stream that reads sensor
                res, _, _, _, _ = vrep.simxReadProximitySensor(clientID, proximity_sensor_3, vrep.simx_opmode_streaming)
            else:
                print('getting proximity sensor 3: call returned with error code: ', res)
    
            # --getting prismatic joint handles
            res, prismatic_joint_1 = vrep.simxGetObjectHandle(clientID, 'Prismatic_joint_1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('prismatic joint 1 handle recieved.')
                # initializing the stream that reads force of joint
                res, _ = vrep.simxGetJointForce(clientID, prismatic_joint_1, vrep.simx_opmode_streaming)
                res, _ = vrep.simxGetJointPosition(clientID, prismatic_joint_1, vrep.simx_opmode_streaming)
            else:
                print('getting prismatic joint 1: call returned with error code: ', res)
    
            res, prismatic_joint_2 = vrep.simxGetObjectHandle(clientID, 'Prismatic_joint_2', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('prismatic joint 2 handle recieved.')
                # initializing the stream that reads force of joint
                res, _ = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_streaming)
                res, _ = vrep.simxGetJointPosition(clientID, prismatic_joint_2, vrep.simx_opmode_streaming)
            else:
                print('getting prismatic joint 2: call returned with error code: ', res)
    
            res, prismatic_joint_3 = vrep.simxGetObjectHandle(clientID, 'Prismatic_joint_3', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('prismatic joint 3 handle recieved.')
                # initializing the stream that reads force of joint
                res, _ = vrep.simxGetJointForce(clientID, prismatic_joint_3, vrep.simx_opmode_streaming)
                res, _ = vrep.simxGetJointPosition(clientID, prismatic_joint_3, vrep.simx_opmode_streaming)
            else:
                print('getting prismatic joint 3: call returned with error code: ', res)
    
            # --getting wheel handles
            res, P_leftWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer left wheel handle recieved.')
                # initializing the stream that reads position of the wheel
                res, _ = vrep.simxGetJointPosition(clientID, P_leftWheel, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer left wheel: call returned with error code:', res)
    
            res, P_rightWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer right wheel handle recieved.')
                # initializing the stream that reads position of the wheel
                res, _ = vrep.simxGetJointPosition(clientID, P_rightWheel, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer right wheel: call returned with error code:', res)
    
            res, P0_leftWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#0 left wheel handle recieved.')
                # initializing the stream that reads position of the wheel
                res, _ = vrep.simxGetJointPosition(clientID, P0_leftWheel, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#0 left wheel: call returned with error code:', res)
    
            res, P0_rightWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#0 right wheel handle recieved.')
                # initializing the stream that reads position of the wheel
                res, _ = vrep.simxGetJointPosition(clientID, P0_rightWheel, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#0 right wheel: call returned with error code:', res)
    
            res, P1_leftWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#1 left wheel handle recieved.')
                # initializing the stream that reads position of the wheel
                res, _ = vrep.simxGetJointPosition(clientID, P1_leftWheel, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#1 left wheel: call returned with error code:', res)
    
            res, P1_rightWheel = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#1 right wheel handle recieved.')
                # initializing the stream that reads position of the wheel
                res, _ = vrep.simxGetJointPosition(clientID, P1_rightWheel, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#1 right wheel: call returned with error code:', res)
    
            # --getting spherical joint handles
            res, P_leftSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_left', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer left spherical joint handle recieved.')
                # initializing the stream that reads position of the spherical joint
                res, _ = vrep.simxGetJointMatrix(clientID, P_leftSphericalJoint, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer left spherical joint: call returned with error code:', res)
    
            res, P_rightSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_right', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer right spherical joint handle recieved.')
                # initializing the stream that reads position of the spherical joint
                res, _ = vrep.simxGetJointMatrix(clientID, P_rightSphericalJoint, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer right spherical joint: call returned with error code:', res)
    
            res, P0_leftSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_left#0', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#0 left spherical joint handle recieved.')
                # initializing the stream that reads position of the spherical joint
                res, _ = vrep.simxGetJointMatrix(clientID, P0_leftSphericalJoint, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#0 left spherical joint: call returned with error code:', res)
    
            res, P0_rightSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_right#0', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#0 right spherical joint handle recieved.')
                # initializing the stream that reads position of the spherical joint
                res, _ = vrep.simxGetJointMatrix(clientID, P0_rightSphericalJoint, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#0 right spherical joint: call returned with error code:', res)
    
            res, P1_leftSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_left#1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#1 left spherical joint handle recieved.')
                # initializing the stream that reads position of the spherical joint
                res, _ = vrep.simxGetJointMatrix(clientID, P1_leftSphericalJoint, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#1 left spherical joint: call returned with error code:', res)
    
            res, P1_rightSphericalJoint = vrep.simxGetObjectHandle(clientID, 'Spherical_right#1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#1 right spherical joint handle recieved.')
                # initializing the stream that reads position of the spherical joint
                res, _ = vrep.simxGetJointMatrix(clientID, P1_rightSphericalJoint, vrep.simx_opmode_streaming)
            else:
                print('getting Pioneer#1 right spherical joint: call returned with error code:', res)
    
            # --getting the revolute upper disk joint handles
            res, P_UpperDiskJoint = vrep.simxGetObjectHandle(clientID, 'Revolute_upperDisk', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('revolute upper disk joint handle recieved.')
                # initializing the stream that reads joint orientation (for tilt)
                res, _ = vrep.simxGetObjectOrientation(clientID, P_UpperDiskJoint, -1, vrep.simx_opmode_streaming)
                # initializing the stream that reads joint position
                res, _ = vrep.simxGetJointPosition(clientID, P_UpperDiskJoint, vrep.simx_opmode_streaming)
            else:
                print('getting revolute upper disk joint: call returned with error code: ', res)
    
            res, P0_UpperDiskJoint = vrep.simxGetObjectHandle(clientID, 'Revolute_upperDisk#0', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('revolute upper disk joint#0 handle recieved.')
                # initializing the stream that reads joint orientation (for tilt)
                eulerAngle = vrep.simxGetObjectOrientation(clientID, P0_UpperDiskJoint, -1, vrep.simx_opmode_streaming)
                # initializing the stream that reads joint position
                res, _ = vrep.simxGetJointPosition(clientID, P0_UpperDiskJoint, vrep.simx_opmode_streaming)
            else:
                print('getting revolute upper disk joint#0: call returned with error code: ', res)
    
            res, P1_UpperDiskJoint = vrep.simxGetObjectHandle(clientID, 'Revolute_upperDisk#1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('revolute upper disk joint#1 handle recieved.')
                # initializing the stream that reads joint orientation (for tilt)
                eulerAngle = vrep.simxGetObjectOrientation(clientID, P1_UpperDiskJoint, -1, vrep.simx_opmode_streaming)
                # initializing the stream that reads joint position
                res, _ = vrep.simxGetJointPosition(clientID, P1_UpperDiskJoint, vrep.simx_opmode_streaming)
            else:
                print('getting revolute upper disk joint#1: call returned with error code: ', res)
    
            # getting the robots' object handles
            res, pioneer = vrep.simxGetObjectHandle(clientID, 'Pioneer', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer recieved.')
                # initializing stream that reads object velocity (for stability parameter)
                res, linearVelocity, _ = vrep.simxGetObjectVelocity(clientID, pioneer, vrep.simx_opmode_streaming)
            else:
                print('getting pioneer: call returned with error code: ', res)
    
            res, pioneer0 = vrep.simxGetObjectHandle(clientID, 'Pioneer#0', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#0 recieved.')
                # initializing stream that reads object velocity (for stability parameter)
                res, linearVelocity, _ = vrep.simxGetObjectVelocity(clientID, pioneer0, vrep.simx_opmode_streaming)
            else:
                print('getting pioneer#0: call returned with error code: ', res)
    
            res, pioneer1 = vrep.simxGetObjectHandle(clientID, 'Pioneer#1', vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                # print ('pioneer#1 recieved.')
                # initializing stream that reads object velocity (for stability parameter)
                res, linearVelocity, _ = vrep.simxGetObjectVelocity(clientID, pioneer1, vrep.simx_opmode_streaming)
            else:
                print('getting pioneer#1: call returned with error code: ', res)
    
            ##---------------------------------------------------------------**** starting the simulation ****----------------------------------------------------------------
            ##----------------------------------------------------------------------------------------------------------------------------------------------------------------
            res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            if res == vrep.simx_return_ok:
                print('Simulation started')
                print('episodeNum:  ', episodeNum)
            else:
                print('Starting Simulation call returned with error code: ', res)
    
            done = False
            batch_size = 120
    
            # scene parameters
            mu = 0.5
            mass = 9.331e-02
    
            # number of steps in each episode
            max_steps = 500
    
            break_loop = 0
            done = 0
            done0 = 0
            done1 = 0
            StepNum = 0
    
            Sum_reward = 0
            Sum_reward0 = 0
            Sum_reward1 = 0
    
            ##--- main loop ---
            while vrep.simxGetConnectionId(clientID) != -1 and break_loop == 0:  # while we are connected to the server...
    
                full_steps += 1
                if StepNum == 0:
                    # getting states
                    state = NNinput('Pioneer')[0:9]
                    state = numpy.reshape(state, [1, state_size])
                    state = preprocessing.normalize(state)
                    state0 = NNinput('Pioneer#0')[0:9]
                    state0 = numpy.reshape(state0, [1, state_size])
                    state0 = preprocessing.normalize(state0)
                    state1 = NNinput('Pioneer#1')[0:9]
                    state1 = numpy.reshape(state1, [1, state_size])
                    state1 = preprocessing.normalize(state1)
    
                StepNum = StepNum + 1
    
                # increasing gamma   decreasing lr
                if full_steps % 20 == 0:
                    if agent.gamma < 0.99:
                        agent.gamma = 1 - 0.98 * (1 - agent.gamma)
                    else:
                        agent.gamma = 0.99
                    if agent.learning_rate > 0.001:
                        agent.learning_rate = 0.98 * agent.learning_rate
                    if batch_size < 800:
                        batch_size += 20
    
                # picking actions and moving the three robots.
                # robots responsible for moving each prismatic:      pris1->Pioneer
                #                                                   pris2->Pioneer#0
                #                                                   pris3->Pioneer#1
                action_bucketNum = agent.act(state)
                action0_bucketNum = agent.act(state0)
                action1_bucketNum = agent.act(state1)
                print('act', action_bucketNum)
                print('act0', action0_bucketNum)
                print('act1', action1_bucketNum)
                action = agent.act_into_realAct(action_size, action_bucketNum)
                action0 = agent.act_into_realAct(action_size, action0_bucketNum)
                action1 = agent.act_into_realAct(action_size, action1_bucketNum)
                moveInput = numpy.zeros(9)
                moveInput[0] = action[0]
                moveInput[1] = action[1]
                moveInput[2] = action0[0]
                moveInput[3] = action0[1]
                moveInput[4] = action1[0]
                moveInput[5] = action1[1]
                moveInput[6] = action[2]
                moveInput[7] = action0[2]
                moveInput[8] = action1[2]
    
                # ropeBehave = checkRopeBehave(moveInput)
                ropeBehave, prisForce1, prisForce2, prisForce3 = move(clientID, moveInput)  # moving to next state
    
                res, Force1 = vrep.simxGetJointForce(clientID, prismatic_joint_1, vrep.simx_opmode_buffer)
                res, Force2 = vrep.simxGetJointForce(clientID, prismatic_joint_2, vrep.simx_opmode_buffer)
                res, Force3 = vrep.simxGetJointForce(clientID, prismatic_joint_3, vrep.simx_opmode_buffer)
                print('force1:', Force1)
                print('force2:', Force2)
                print('force3:', Force3)
    
                # rope for testing checkropebehave
                if (ropeBehave[0] == 1 and prisForce1 <= -0.1) or (ropeBehave[0] == 0 and prisForce1 >= 0.1):
                    rope.append(1)
                elif (ropeBehave[0] == 1 and prisForce1 >= 0.1):
                    rope.append(0)
                elif (ropeBehave[0] == 0 and prisForce1 != 0):
                    rope.append(-1)
                if prisForce1 >= 0.1:
                    positive += 1
                if ropeBehave[0] == 1:
                    negitive += 1
    
                if (ropeBehave[1] == 1 and prisForce2 <= -0.1) or (ropeBehave[1] == 0 and prisForce2 >= 0.1):
                    rope.append(1)
                elif (ropeBehave[1] == 1 and prisForce2 >= 0.1):
                    rope.append(0)
                elif (ropeBehave[1] == 0 and prisForce2 != 0):
                    rope.append(-1)
                if prisForce2 >= 0.1:
                    positive += 1
                if ropeBehave[1] == 1:
                    negitive += 1
    
                if (ropeBehave[2] == 1 and prisForce3 <= -0.1) or (ropeBehave[2] == 0 and prisForce3 >= 0.1):
                    rope.append(1)
                elif (ropeBehave[2] == 1 and prisForce3 >= 0.1):
                    rope.append(0)
                elif (ropeBehave[2] == 0 and prisForce3 != 0):
                    rope.append(-1)
                if prisForce3 >= 0.1:
                    positive += 1
                if ropeBehave[2] == 1:
                    negitive += 1
    
                # getting next_state, reward, and done
                next_state = NNinput('Pioneer')[0:9]
                next_state = numpy.reshape(next_state, [1, state_size])
                next_state = preprocessing.normalize(next_state)
                next_state0 = NNinput('Pioneer#0')[0:9]
                next_state0 = numpy.reshape(next_state0, [1, state_size])
                next_state0 = preprocessing.normalize(next_state0)
                next_state1 = NNinput('Pioneer#1')[0:9]
                next_state1 = numpy.reshape(next_state1, [1, state_size])
                next_state1 = preprocessing.normalize(next_state1)
    
                isStopped, time = StopSimulation(clientID, proximity_sensor_1, proximity_sensor_2, proximity_sensor_3,
                                                 prismatic_joint_1, prismatic_joint_2, prismatic_joint_3)
                if isStopped != 0:
                    break_loop = 1
                    flipped = 1
    
                if StepNum % 10 >= 7 and StepNum % 10 <= 9:  # checking the algorithm
                    q_pred = agent.predict(state)
                    q0_pred = agent.predict(state0)
                    q1_pred = agent.predict(state1)
    
                    # print('Q',q_pred)
                    # print('Q0',q0_pred)
                    # print('Q1',q1_pred)
    
                if isStopped == 0 or StepNum >= max_steps:
                    print('StepNum:', StepNum, '    fullSteps:', full_steps, '     gamma:', agent.gamma, '    lr:',
                          agent.learning_rate)
                    SP = stabilityParam(clientID, pioneer, pioneer0, pioneer1, prismatic_joint_1, prismatic_joint_2,
                                        prismatic_joint_3, P_leftSphericalJoint, P_rightSphericalJoint,
                                        P0_leftSphericalJoint, P0_rightSphericalJoint, P1_leftSphericalJoint,
                                        P1_rightSphericalJoint, 'Pioneer', mass, mu)
                    reward = SP
                    SP = stabilityParam(clientID, pioneer, pioneer0, pioneer1, prismatic_joint_1, prismatic_joint_2,
                                        prismatic_joint_3, P_leftSphericalJoint, P_rightSphericalJoint,
                                        P0_leftSphericalJoint, P0_rightSphericalJoint, P1_leftSphericalJoint,
                                        P1_rightSphericalJoint, 'Pioneer#0', mass, mu)
                    reward0 = SP
                    SP = stabilityParam(clientID, pioneer, pioneer0, pioneer1, prismatic_joint_1, prismatic_joint_2,
                                        prismatic_joint_3, P_leftSphericalJoint, P_rightSphericalJoint,
                                        P0_leftSphericalJoint, P0_rightSphericalJoint, P1_leftSphericalJoint,
                                        P1_rightSphericalJoint, 'Pioneer#1', mass, mu)
                    reward1 = SP
    
                    Sum_reward += reward
                    Sum_reward0 += reward0
                    Sum_reward1 += reward1
    
                tippingPunishment = 100  # this should be checked
                if isStopped != 0:
                    reward = reward - tippingPunishment
                    reward0 = reward0 - tippingPunishment
                    reward1 = reward1 - tippingPunishment
                    print(reward, '     ', reward0, '    ', reward1)
    
                if flipped == 1:
                    f = open(filename, "a+")
                    f.write('flipped at step: ' + str(StepNum) + '\n')
                    f.write('SumRewards: ' + str(Sum_reward) + ' ' + str(Sum_reward0) + ' ' + str(Sum_reward1) + '\n')
                    f.write('\n')
                    f.close()
                    Sum_reward = 0
                    Sum_reward0 = 0
                    Sum_reward1 = 0
    
                if isStopped != 0:
                    end_step.append(StepNum)
                    StepNum = 0
                    end_time.append(time / 1000)
                    print('end_time:           ', time / 1000)
    
                if StepNum >= max_steps:
                    StepNum = 0
                    reward += 100  # goal
                    reward0 += 100
                    reward1 += 100
                    done = 1
                    done0 = 1
                    done1 = 1
                    res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    
                # remember the previous state,action (the bucket number of action is saved, not real action),reward and done.
                print('rewards:', reward, '     ', reward0, '    ', reward1)
                agent.remember(state, action_bucketNum, reward, next_state, done)
                agent.remember(state0, action0_bucketNum, reward0, next_state0, done0)
                agent.remember(state1, action1_bucketNum, reward1, next_state1, done1)
    
                # testing Qnetwork
                if StepNum % 10 == 9:
    
                    # real_reward = Q
                    Num = action_bucketNum[2] - 1 + 5 * (action_bucketNum[1] - 1) + 25 * (action_bucketNum[0] - 1)
                    print('action_Num', Num)
                    pred_Q = q_pred[0][int(Num)]
    
                    # real_reward0 = Q0
                    Num0 = action0_bucketNum[2] - 1 + 5 * (action0_bucketNum[1] - 1) + 25 * (action0_bucketNum[0] - 1)
                    print('action_Num', Num0)
                    pred_Q0 = q0_pred[0][int(Num0)]
    
                    # real_reward1 = Q1
                    Num1 = action1_bucketNum[2] - 1 + 5 * (action1_bucketNum[1] - 1) + 25 * (action1_bucketNum[0] - 1)
                    print('action_Num', Num1)
                    pred_Q1 = q1_pred[0][int(Num1)]
    
                    print('action:  ', action, action0, action1)
                    print('action_bucket', action_bucketNum, action0_bucketNum, action1_bucketNum)
                    print('state', state)
                    print('real reward:    ', reward, reward0, reward1)
                    print('pred_Q:     ', pred_Q, pred_Q0, pred_Q1)
                    # print('percent: ',(pred_reward-reward)/reward*100,(pred_reward0-reward0)/reward0*100,(pred_reward1-reward1)/reward1*100)
    
                    loss = check_loss(state, next_state, Num, reward, agent)
                    loss0 = check_loss(state0, next_state0, Num0, reward0, agent)
                    loss1 = check_loss(state1, next_state1, Num1, reward1, agent)
                    print('loss: ', loss, loss0, loss1)
    
                    if full_steps < 3500:
                        all_loss[loss_num] = loss
                        all_loss0[loss_num] = loss0
                        all_loss1[loss_num] = loss1
                        loss_num += 1
                    else:
                        numpy.save('Loss_num', all_loss)
                        numpy.save('Loss0_num', all_loss0)
                        numpy.save('Loss1_num', all_loss1)
    
                    rope_count = 0
                    rope_positive = 0
                    for x in rope:
                        if x == 1:
                            rope_count += 1
                        elif x == 0:
                            rope_positive += 1
    
                    # saving to file
                    f = open(filename, "a+")
                    f.write('step: ' + str(full_steps) + ' gamma: ' + str(agent.gamma) + ' lr: ' + str(
                        agent.learning_rate) + '\n')
                    f.write('rewards: ' + str(reward) + ' ' + str(reward0) + ' ' + str(reward1) + '\n')
                    f.write('pred_Qs: ' + str(pred_Q) + ' ' + str(pred_Q0) + ' ' + str(pred_Q1) + '\n')
                    f.write('loss: ' + str(loss) + ' ' + str(loss0) + ' ' + str(loss1) + '\n')
                    if negitive > 0:
                        f.write('false_negitive: ' + str(rope_positive / negitive * 100) + '\n')
                    f.write('\n')
                    f.close()
    
                # make next_state the new current state for the next frame
                state = next_state
                state0 = next_state0
                state1 = next_state1
    
                # break if done
                if done == 1:
                    StepNum = 0
                    break_loop = 1
    
                # train dnn
                if len(agent.memory) > batch_size:
                    agent.replay(batch_size)
    
                if full_steps >= 4000:
                    agent.save('D:/internship/robot final files/final/finalDQN11new')
                    print('losss nummmmmmmmm:', loss_num)
    
                # saving scene
                # if StepNum==100:
                # res = vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot_wait)  #This line saves the scene but this changes the current scene
    
            # saving model
            agent.save('D:/internship/robot final files/final/finalDQN')
            # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
            vrep.simxGetPingTime(clientID)
    
    else:
        print('Failed connecting to remote API server')
    print('Program ended')
    
    vrep.simxFinish(clientID)
    
