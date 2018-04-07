function  s = collision_line(S, p_robot,r_robot, p_obstacle, r_obstacle, theta_start, theta_goal)
[~,Size] = size(theta_start);
c = zeros(1,Size);
s = zeros(1,Size);
[~,m] = size(p_obstacle);

for i = 1:Size
while c(i) == 0
theta = (1-s(i))*theta_start(:,i) + s(i)*theta_goal(:,i);
p = position(S,theta, p_robot);
p_base_in_world = [0;0;0];
p1 = p(:,1);
p2 = p(:,2);
p3 = p(:,3);
p4 = p(:,4);
p5 = p(:,5);

p1_1 = p1+(p2-p1)/4*1;
p1_2 = p1+(p2-p1)/4*2;
p1_3 = p1+(p2-p1)/4*3;
p3_1 = (p4+p3)/2;
p_ = [p_base_in_world   p1 p1_1 p1_2 p1_3   p2   p3 p3_1   p4   p5];
r = [           0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05];
[~,n] = size(p);
for j = 1:n
    for k = 1:m
        if collision(p_(:,j), p_obstacle(:,k), r(j), r_obstacle(k)) == 1
            c(i) = 1;
        end
    end
end
s(i) = s(i) + 0.01;
if s(i)>1
    break;
end
end
if s(i)>1
    s(i) = 0;
end
end
end