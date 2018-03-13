import vrep
import time
import numpy as np

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

### =============================================================================
# Get "handle" to all joints of robot
### =============================================================================
# Get "handle" to the first joint of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'Mico_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')
# Get "handle" to the second joint of robot
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'Mico_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for second joint')
# Get "handle" to the third joint of robot
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'Mico_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for third joint')
# Get "handle" to the fourth joint of robot
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'Mico_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fourtn joint')
# Get "handle" to the fifth joint of robot
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'Mico_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')
# Get "handle" to the sixth joint of robot
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'Mico_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')

### =============================================================================                                               
# Start simulation
### =============================================================================
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

### =============================================================================
# Wait two seconds
### =============================================================================
time.sleep(2)

### =============================================================================
# Get the values of all joint variables
### =============================================================================
# Get the current value of the first joint variable
result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta1 = {:f}'.format(theta1))
# Get the current value of the first joint variable
result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get second joint variable')
print('current value of second joint variable: theta2 = {:f}'.format(theta2))
# Get the current value of the first joint variable
result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get third joint variable')
print('current value of third joint variable: theta3 = {:f}'.format(theta3))
# Get the current value of the first joint variable
result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get fourth joint variable')
print('current value of fourth joint variable: theta4 = {:f}'.format(theta4))
# Get the current value of the first joint variable
result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get fifth joint variable')
print('current value of fifth joint variable: theta5 = {:f}'.format(theta5))
# Get the current value of the first joint variable
result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sixth joint variable')
print('current value of sixth joint variable: theta6 = {:f}'.format(theta6))

### =============================================================================
# Set the desired values of joints variable
### =============================================================================
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1 + (np.pi / 2), vrep.simx_opmode_oneshot)
# Set the desired value of the second joint variable
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta2 + (np.pi / 2), vrep.simx_opmode_oneshot)
# Set the desired value of the third joint variable
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3 + (np.pi / 2), vrep.simx_opmode_oneshot)
# Set the desired value of the fourth joint variable
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta4 + (np.pi / 2), vrep.simx_opmode_oneshot)
# Set the desired value of the fifth joint variable
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta5 + (np.pi / 2), vrep.simx_opmode_oneshot)
# Set the desired value of the sixth joint variable
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta6 + (np.pi / 2), vrep.simx_opmode_oneshot)


### =============================================================================
# Wait two seconds
### =============================================================================
time.sleep(2)

### =============================================================================
# Get the values of all joint variables
### =============================================================================
# Get the current value of the first joint variable
result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta1 = {:f}'.format(theta1))
# Get the current value of the first joint variable
result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get second joint variable')
print('current value of second joint variable: theta2 = {:f}'.format(theta2))
# Get the current value of the first joint variable
result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get third joint variable')
print('current value of third joint variable: theta3 = {:f}'.format(theta3))
# Get the current value of the first joint variable
result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get fourth joint variable')
print('current value of fourth joint variable: theta4 = {:f}'.format(theta4))
# Get the current value of the first joint variable
result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get fifth joint variable')
print('current value of fifth joint variable: theta5 = {:f}'.format(theta5))
# Get the current value of the first joint variable
result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sixth joint variable')
print('current value of sixth joint variable: theta6 = {:f}'.format(theta6))

### =============================================================================                                               
# Stop simulation
### =============================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
