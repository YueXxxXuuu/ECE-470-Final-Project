function [result] = forward(theta)
t1 = theta(1);
t2 = theta(2);
t3 = theta(3);
t4 = theta(4);
t5 = theta(5);
t6 = theta(6);

q1 = -t1;
q2 = t2+pi/2.0;
q3 = t3-pi/2.0;
q4 = t4;
q5 = t5;
q6 = t6 - 100.0*pi/180.0;


D1 = 0.2755-8.1581e-02;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.0743;
D5 = 0.0743;
D6 = 0.1687;
e2 = 0.0098;

aa = 11.0*pi/72.0;
sa = sin(aa);
s2a = sin(2.0*aa);

d4b = (D3 + sa/s2a*D4);
d5b = (sa/s2a*D4+sa/s2a*D5);
d6b = (sa/s2a*D5+D6);



param.a = [0;D2;0;0;0;0];
param.d = [D1;0;-e2;-d4b;-d5b;-d6b];
param.alp = [pi/2.0;pi;pi/2.0;2.0*aa;2.0*aa;pi];
param.th = [q1;q2;q3;q4;q5;q6];
param.type = ['r';'r';'r';'r';'r';'r'];

[~,result] = forwardKinematics(param.a,param.d,param.alp,param.th);
end

