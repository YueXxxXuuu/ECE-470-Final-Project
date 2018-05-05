function move_robot(next, prev, joint_handles, vrep, clientID)
init_theta = pi;
ArmJoints = ['Jaco_joint1','Jaco_joint2','Jaco_joint3',...
             'Jaco_joint4','Jaco_joint5','Jaco_joint6'];

for a = 1:10
joint_angles = a/10*next + (10-a)/10*prev;
for i = 0:5
    [result, joint_handles(i+1)] = vrep.simxGetObjectHandle(clientID, ArmJoints(11*i+1:11*i+11), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        sprintf('could not get joint handle%d = %d', i,ArmJoints(11*i+1:11*i+11))
    end
end
%%% Iterate the following code block 6 times to move each joint individually
for i = 1:6
    curr_handle = joint_handles(i);
	%%% Set the desired value of the current joint variable
	vrep.simxSetJointTargetPosition(clientID, curr_handle, init_theta + joint_angles(i), vrep.simx_opmode_oneshot);
	%%% Wait two seconds
end
end
end