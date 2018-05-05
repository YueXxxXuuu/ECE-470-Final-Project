function pick(vrep,clientID)
pause(1);
[result,j0] = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor1',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle for JacoHand_fingers12_motor1")
end
[result,j1] = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor2',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle JacoHand_fingers12_motor2")
end
[result,j2] = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor1',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle JacoHand_finger3_motor1")
end
[result,j3] = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor2',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle JacoHand_finger3_motor2")
end

    result = vrep.simxSetJointTargetVelocity(clientID,j0,-0.01,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_fingers12_motor1")
    end
    result = vrep.simxSetJointTargetVelocity(clientID,j1,-0.005,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_fingers12_motor2")
    end
    result = vrep.simxSetJointTargetVelocity(clientID,j2,-0.01,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_finger3_motor1")
    end
    result = vrep.simxSetJointTargetVelocity(clientID,j3,-0.01,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_finger3_motor1")
    end
    pause(3);
end