function  s = collision_line(S, T_base_in_world, p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal)
[~,Size] = size(theta_start);
c = zeros(1,Size);
s = zeros(1,Size);
[~,m] = size(p_obstacle);

for i = 1:Size
while c(i) == 0
theta = (1-s(i))*theta_start(:,i) + s(i)*theta_goal(:,i);
p_base_in_world = [0;0;0];
T_result = forward(theta);
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

p1_1 = p1+(p2-p1)/4*1;
p1_2 = p1+(p2-p1)/4*2;
p1_3 = p1+(p2-p1)/4*3;
p3_1 = (p4+p3)/2;
p_ = [p_base_in_world   p1 p1_1 p1_2 p1_3   p2   p3 p3_1   p4   p5];
r = [           0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05];
[~,n] = size(p_);
for j = 1:n
    for k = 1:m
        if collision(p_(:,j), p_obstacle(:,k), r(j), r_obstacle(k)) == 1
            c(i) = 1;
        end
    end
end
s(i) = s(i) + 0.05;
if s(i)>1
    break;
end
end
if s(i)>1
    s(i) = 0;
end
end
end