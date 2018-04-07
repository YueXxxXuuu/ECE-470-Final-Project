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

%====================================================================
% Forward Kinematis
%====================================================================
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
T_base_in_world = [R_base_in_world, p_base_in_world;0,0,0,1];
T_base_in_world = vpa(T_base_in_world,4);

p_obstacle1 = [0.5;0.3;0.2];
p_obstacle2 = [0.2;0.3;0.5];
p_obstacle3 = [-0.2;-0.5;0.3];
p_obstacle = [p_obstacle1 p_obstacle2 p_obstacle3];
r_obstacle = [       0.05        0.13        0.16];
theta_start = [0;0;0;0;0;0];
theta_goal = [5.4; 1.7; 0.5; 2.10; 2.22; 1.79];
%theta_goal = [1; 2; 1; 3; 2; 2];
%theta_goal = [-0.4; 1.6; 0.5; 3; 2; 2];

T_result = forward(theta_goal);
Tfin = T_base_in_world*T_result(:,:,6);
pfin = vpa(Tfin(1:3,4),5);


%%% set the Dummy Point
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end
result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p_obstacle1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

%%% set the Dummy0 Point
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy0', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end
result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p_obstacle2,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

%%% set the Dummy1 Point
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy1', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end
result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,p_obstacle3,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

%%% set the Dummy2 Point
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy2', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end
result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,pfin,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end


s = path_planning(T_base_in_world, p_obstacle, r_obstacle, theta_start, theta_goal);
if s == 0
    disp("no path");
end 

init_theta = pi;
[~, Size] = size(s);

for i = 1:Size
    joint_angles = s(:,i);
    c = collision_self(joint_angles);
    if c == 1
        disp("GG");
    end    
end 


%====================================================================
% Get "handle" to joints of robot
%====================================================================
for j = 2:Size
s_curr = s(:,j);
s_prev = s(:,j-1);
s_set = zeros(6,50);
for x = 1:6
s_set(x,:) = linspace(s_prev(x),s_curr(x),50);
end
for k = 2:50
joint_angles = s_set(:,k);
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

pause(3)
%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);
