function [theta] = inverseKinematics(T_1in0, M , S)
I = eye(6);
theta = zeros(6,1);
i = 1;
k = 0;

while i>=0.01
    T = eye(4);
    for i = 1:6
        currentS = S(:,i);
        currentT = expm(bra(currentS)*theta(i));
        T = T*currentT;
    end
    T = T*M;
 
    J = S(:,1);

    tempT = eye(4);
    for i = 2:6
        currentS = S(:,i-1);
        currentT = expm(bra(currentS)*theta(i-1));
        tempT = tempT*currentT;
        SS = adj(tempT);
        currentJ = SS*S(:,i);
        J = [J currentJ];
    end
    V = logm(T_1in0*inv(T));
    V = vpa(V,4);
    V_b = adj(T)\unbra(V);
    V_b = vpa(V_b,4);
    thetad = (J.'*J+0.01*I)\J.'*unbra(V);
    thetad = vpa(thetad,4);
    theta = theta+((thetad)*1);
    theta = vpa(theta,4);
    i = norm(V_b);
    disp(i);
    k = k + 1;
    if k > 30
       disp('Cannot find a solution');
       disp('error appears');
       theta = [];
       %====================================================================
       % Stop Simulation & Finish session
       %====================================================================
       vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
       vrep.simxGetPingTime(clientID);
       vrep.simxFinish(clientID);
       return;
    end
end
end
 
function A = adj(T)
R = [T(1,1) T(1,2) T(1,3);T(2,1) T(2,2) T(2,3);T(3,1) T(3,2) T(3,3)];
P = [T(1,4);T(2,4);T(3,4)];
O = zeros(3,3);
A = [R O;skew(P)*R R];
end
 
function A =bra(T)
O = [0,0,0];
I = 0;
w = [T(1);T(2);T(3)];
W = skew(w);
v = [T(4);T(5);T(6)];
A = [W v;O I];
end
 
function A =unbra(T)
W = [T(1,1) T(1,2) T(1,3);T(2,1) T(2,2) T(2,3);T(3,1) T(3,2) T(3,3)];
w = unskew(W);
v = [T(1,4);T(2,4);T(3,4)];
A = [w;v];
end
 
function A = skew(T)
A = [0 -T(3) T(2);T(3) 0 -T(1);-T(2) T(1) 0];
end
 
function A = unskew(T)
A = [T(3,2);T(1,3);T(2,1)];
end

