%====================================================================
% My Variables
%====================================================================
ArmJoints = ['Jaco_joint1','Jaco_joint2','Jaco_joint3',...
             'Jaco_joint4','Jaco_joint5','Jaco_joint6'];
joint_handles = zeros(1,6);
JointAngles = zeros(1,6);
init_theta = pi;


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
S = vpa(S,4);

R_start = getR([deg2rad(-1.1000e+02),deg2rad(-9.3915e-06),deg2rad(+8.7000e+00)]) ;
p_start = [-6.0350e-02; +3.0171e-01; +8.9924e-01];
T_start = [R_start, p_start; 0,0,0,1];


p1_1 = [0.3;0.5;0.1];
M = [1,0,0,p1_1(1);0,-1,0,p1_1(2);0,0,-1,p1_1(3);0,0,0,1];
a1_1 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a1_1(i) < -pi
       a1_1(i) = a1_1(i) + 2*pi;
   end
   while  a1_1(i) > pi
       a1_1(i) = a1_1(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p1_2 = [0.3;0.5;0.2];
M = [1,0,0,p1_2(1);0,-1,0,p1_2(2);0,0,-1,p1_2(3);0,0,0,1];
a1_2 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a1_2(i) < -pi
       a1_2(i) = a1_2(i) + 2*pi;
   end
   while  a1_2(i) > pi
       a1_2(i) = a1_2(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p1_3 = [0.3;0.5;0.3];
M = [1,0,0,p1_3(1);0,-1,0,p1_3(2);0,0,-1,p1_3(3);0,0,0,1];
a1_3 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a1_3(i) < -pi
       a1_3(i) = a1_3(i) + 2*pi;
   end
   while  a1_3(i) > pi
       a1_3(i) = a1_3(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p1_4 = [0.3;0.5;0.35];
M = [1,0,0,p1_4(1);0,-1,0,p1_4(2);0,0,-1,p1_4(3);0,0,0,1];
a1_4 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a1_4(i) < -pi
       a1_4(i) = a1_4(i) + 2*pi;
   end
   while  a1_4(i) > pi
       a1_4(i) = a1_4(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p2_1 = [0.3;0.3;0.1];
M = [1,0,0,p2_1(1);0,-1,0,p2_1(2);0,0,-1,p2_1(3);0,0,0,1];
a2_1 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a2_1(i) < -pi
       a2_1(i) = a2_1(i) + 2*pi;
   end
   while  a2_1(i) > pi
       a2_1(i) = a2_1(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p2_2 = [0.3;0.3;0.2];
M = [1,0,0,p2_2(1);0,-1,0,p2_2(2);0,0,-1,p2_2(3);0,0,0,1];
a2_2 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a2_2(i) < -pi
       a2_2(i) = a2_2(i) + 2*pi;
   end
   while  a2_2(i) > pi
       a2_2(i) = a2_2(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p2_3 = [0.3;0.3;0.3];
M = [1,0,0,p2_3(1);0,-1,0,p2_3(2);0,0,-1,p2_3(3);0,0,0,1];
a2_3 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a2_3(i) < -pi
       a2_3(i) = a2_3(i) + 2*pi;
   end
   while  a2_3(i) > pi
       a2_3(i) = a2_3(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p2_4 = [0.3;0.3;0.4];
M = [1,0,0,p2_4(1);0,-1,0,p2_4(2);0,0,-1,p2_4(3);0,0,0,1];
a2_4 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a2_4(i) < -pi
       a2_4(i) = a2_4(i) + 2*pi;
   end
   while  a2_4(i) > pi
       a2_4(i) = a2_4(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3_1 = [0.5;0.3;0.1];
M = [1,0,0,p3_1(1);0,-1,0,p3_1(2);0,0,-1,p3_1(3);0,0,0,1];
a3_1 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a3_1(i) < -pi
       a3_1(i) = a3_1(i) + 2*pi;
   end
   while  a3_1(i) > pi
       a3_1(i) = a3_1(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3_2 = [0.5;0.3;0.2];
M = [1,0,0,p3_2(1);0,-1,0,p3_2(2);0,0,-1,p3_2(3);0,0,0,1];
a3_2 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a3_2(i) < -pi
       a3_2(i) = a3_2(i) + 2*pi;
   end
   while  a3_2(i) > pi
       a3_2(i) = a3_2(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3_3 = [0.5;0.3;0.3];
M = [1,0,0,p3_3(1);0,-1,0,p3_3(2);0,0,-1,p3_3(3);0,0,0,1];
a3_3 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a3_3(i) < -pi
       a3_3(i) = a3_3(i) + 2*pi;
   end
   while  a3_3(i) > pi
       a3_3(i) = a3_3(i) - 2*pi;
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3_4 = [0.4;0.3;0.4];
M = [1,0,0,p3_4(1);0,-1,0,p3_4(2);0,0,-1,p3_4(3);0,0,0,1];
a3_4 = real(inverseKinematics(M,T_start,S));
for i= 1:6
   while  a3_4(i) < -pi
       a3_4(i) = a3_4(i) + 2*pi;
   end
   while  a3_4(i) > pi
       a3_4(i) = a3_4(i) - 2*pi;
   end
end
%%
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


%%% Get the R and p for the base frame (Jaco) w.r.t the world frame
%%% Get the handle of Jaco object
[result, jaco_handle] = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco handle')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move_robot1(a1_4, zeros(6,1), joint_handles, vrep, clientID);
move_robot(a1_3, a1_4, joint_handles, vrep, clientID);
pick(vrep,clientID);
move_robot(a1_4, a1_3, joint_handles, vrep, clientID);

move_robot1(a3_4, a1_4, joint_handles, vrep, clientID);
move_robot(a3_1, a3_4, joint_handles, vrep, clientID);
drop(vrep,clientID);
move_robot(a3_4, a3_1, joint_handles, vrep, clientID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move_robot1(a1_4, a3_4, joint_handles, vrep, clientID);
move_robot(a1_2, a1_4, joint_handles, vrep, clientID);
pick(vrep,clientID);
move_robot(a1_4, a1_2, joint_handles, vrep, clientID);

move_robot1(a2_4, a1_4, joint_handles, vrep, clientID);
move_robot(a2_1, a2_4, joint_handles, vrep, clientID);
drop(vrep,clientID);
move_robot(a2_4, a2_1, joint_handles, vrep, clientID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move_robot1(a3_4, a2_4, joint_handles, vrep, clientID);
move_robot(a3_1, a3_4, joint_handles, vrep, clientID);
pick(vrep,clientID);
move_robot(a3_4, a3_1, joint_handles, vrep, clientID);

move_robot1(a2_4, a3_4, joint_handles, vrep, clientID);
move_robot(a2_2, a2_4, joint_handles, vrep, clientID);
drop(vrep,clientID);
move_robot(a2_4, a2_2, joint_handles, vrep, clientID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move_robot1(a1_4, a2_4, joint_handles, vrep, clientID);
move_robot(a1_1, a1_4, joint_handles, vrep, clientID);
pick(vrep,clientID);
move_robot(a1_4, a1_1, joint_handles, vrep, clientID);

move_robot1(a3_4, a1_4, joint_handles, vrep, clientID);
move_robot(a3_1, a3_4, joint_handles, vrep, clientID);
drop(vrep,clientID);
move_robot(a3_4, a3_1, joint_handles, vrep, clientID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move_robot1(a2_4, a3_4, joint_handles, vrep, clientID);
move_robot(a2_2, a2_4, joint_handles, vrep, clientID);
pick(vrep,clientID);
move_robot(a2_4, a2_2, joint_handles, vrep, clientID);

move_robot1(a1_4, a2_4, joint_handles, vrep, clientID);
move_robot(a1_1, a1_4, joint_handles, vrep, clientID);
drop(vrep,clientID);
move_robot(a1_4, a1_1, joint_handles, vrep, clientID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move_robot1(a2_4, a1_4, joint_handles, vrep, clientID);
move_robot(a2_1, a2_4, joint_handles, vrep, clientID);
pick(vrep,clientID);
move_robot(a2_4, a2_1, joint_handles, vrep, clientID);

move_robot1(a3_4, a2_4, joint_handles, vrep, clientID);
move_robot(a3_2, a3_4, joint_handles, vrep, clientID);
drop(vrep,clientID);
move_robot(a3_4, a3_2, joint_handles, vrep, clientID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move_robot1(a1_4, a3_4, joint_handles, vrep, clientID);
move_robot(a1_1, a1_4, joint_handles, vrep, clientID);
pick(vrep,clientID);
move_robot(a1_4, a1_1, joint_handles, vrep, clientID);

move_robot1(a3_4, a1_4, joint_handles, vrep, clientID);
move_robot(a3_3, a3_4, joint_handles, vrep, clientID);
drop(vrep,clientID);
move_robot(a3_4, a3_3, joint_handles, vrep, clientID);


%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);



