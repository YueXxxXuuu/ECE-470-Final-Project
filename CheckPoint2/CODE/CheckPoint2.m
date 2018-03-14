clear;
clc;

%====================================================================
% My Variables
%====================================================================
ArmJoints = ['Jaco_joint1','Jaco_joint2','Jaco_joint3',...
             'Jaco_joint4','Jaco_joint5','Jaco_joint6'];
joint_handles = zeros(1,6);
JointAngles = zeros(1,6);
%====================================================================
% Initialize session and Start Simulation
%====================================================================
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
end
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
pause(2)

%====================================================================
% Forward Kinematis
%====================================================================
%%% A set of joint angles are given
joint_1_angle = pi;
joint_2_angle = pi/2;
joint_3_angle = pi/2;
joint_4_angle = pi/2;
joint_5_angle = pi;
joint_6_angle = pi/2;
joint_angles = [joint_1_angle,joint_2_angle,joint_3_angle,joint_4_angle,joint_5_angle,joint_6_angle];

%%% Get the R and p for the base frame (Jaco) w.r.t the world frame
%%% Get the handle of Jaco object
[result, jaco_handle] = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco handle')
end

%%% Get the position of base frame w.r.t the world frame
[result, p] = vrep.simxGetObjectPosition(clientID,jaco_handle,-1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco position')
end

%%% Get the orientation of base w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, jaco_handle, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco orientation')
end

R_base_in_world = getR(euler_angles);
p_base_in_world = p';

%%%derive the forward kinematics
T_base_in_world = [R_base_in_world, p_base_in_world; 0,0,0,1];
T_result = forward(joint_angles);
T1 = T_base_in_world*T_result(:,:,1);
p1 = vpa(T1(1:3,4),5);
T2 = T_base_in_world*T_result(:,:,2);
p2 = vpa(T2(1:3,4),5);
T3 = T_base_in_world*T_result(:,:,3);
p3 = vpa(T3(1:3,4),5);
T4 = T_base_in_world*T_result(:,:,4);
p4 = vpa(T4(1:3,4),5);
T5 = T_base_in_world*T_result(:,:,5);
p5 = vpa(T5(1:3,4),5);
T6 = T_base_in_world*T_result(:,:,6);
p6 = vpa(T6(1:3,4),5);

%%% set the predict postion of first Joint
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end

pause(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

pause(1)

%%% set the predict postion of second Joint
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy0', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end

pause(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p2,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

pause(1)

%%% set the predict postion of third Joint
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy1', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end

pause(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p3,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

pause(1)

%%% set the predict postion of forth Joint
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy2', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end

pause(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p4,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

pause(1)

%%% set the predict postion of fifth Joint
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy3', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end

pause(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p5,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

pause(1)

%%% set the predict postion of sixth Joint
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy4', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end

pause(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p6,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

pause(1)



%====================================================================
% Get "handle" to joints of robot
%====================================================================
for i = 0:5
    [result, joint_handles(i+1)] = vrep.simxGetObjectHandle(clientID, ArmJoints(11*i+1:11*i+11), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        sprintf('could not get joint handle%d = %d', i,ArmJoints(11*i+1:11*i+11))
    end
end

pause(1)

%%% Iterate the following code block 6 times to move each joint individually
for i = 1:6
    
    curr_handle = joint_handles(i);

	%%% Get the current value of the current joint
	[result, theta] = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking);
	if result ~= vrep.simx_return_ok
		sprintf('could not get joint variable: theta%d', i)
    end

	%%% Wait two seconds
	pause(1)

	%%% Set the desired value of the current joint variable
	vrep.simxSetJointTargetPosition(clientID, curr_handle, theta + joint_angles(i), vrep.simx_opmode_oneshot);

	%%% Wait two seconds
	pause(1)

	%%% Get the new value of the current joint
	[result, theta] = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking);
	if result ~= vrep.simx_return_ok
		sprintf('could not get joint variable %d',i)
    end
    JointAngles(i) = theta;
    %sprintf('current value of joint variable: theta%d = %d', i,theta)    
end

pause(1)

%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);
