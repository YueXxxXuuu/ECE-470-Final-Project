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
%%% Get the orientation of base w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, jaco_handle, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco orientation')
end

R_base_in_world = getR(euler_angles);
p_base_in_world = p';
T_base_in_world = [R_base_in_world, p_base_in_world;0,0,0,1];
T_base_in_world = vpa(T_base_in_world,4);

%%% First, find the screw axes for all 6 joints
%%% a,q without fk are w.r.t the base frame
%%% a_fk and q_fk are w.r.t the world frame
a1 =  getR( [deg2rad(-1.8000e+02),deg2rad(0),deg2rad(0)]) ;
a1 = a1(1:3,3);
q1 = [-5.0000e-02,+1.8190e-12,+1.5675e-01]';
S1 = revolute(a1,q1);

a2 =  getR( [deg2rad(-9.0000e+01),deg2rad(0),deg2rad(0)]) ;
a2 = a2(1:3,3);
q2 = [-5.0000e-02,+0.0000e+00,+2.7550e-01]';
S2 = revolute(a2,q2);

a3 = getR( [deg2rad(9.0000e+01),deg2rad(0),deg2rad(0)]) ;
a3 = a3(1:3,3);
q3 = [-5.0000e-02,-3.7253e-09,+6.8550e-01]';
S3 = revolute(a3,q3);

a4 = getR( [deg2rad(1.8000e+02),deg2rad(0),deg2rad(0)]) ;
a4 = a4(1:3,3);
q4 = [-5.0000e-02,+9.8001e-03,+8.9280e-01]';
S4 = revolute(a4,q4);

a5 = getR( [deg2rad(1.2500e+02),deg2rad(0),deg2rad(0)]) ;
a5 = a5(1:3,3);
q5 = [-5.0000e-02,+4.4049e-02,+9.5863e-01]';
S5 = revolute(a5,q5);

a6 = getR([deg2rad(7.0000e+01),deg2rad(0),deg2rad(0)]) ;
a6 = a6(1:3,3);
q6 = [-5.0000e-02,+1.1771e-01,+9.6839e-01]';
S6 = revolute(a6,q6);

S = [S1,S2,S3,S4,S5,S6];
p_obstacle1 = [0.5;0.3;0.2];
p_obstacle2 = [0.2;0.3;0.5];
p_obstacle3 = [-0.2;-0.5;0.3];
p_robot = [q1 q2 q3 q4 q5 q6];
r_robot = [0 0 0 0 0 0];
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


s = path_planning(S, T_base_in_world, p_robot,r_robot, p_obstacle, r_obstacle, theta_start, theta_goal);
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
