function  p = position(S, theta, p_robot)
 [~, n] = size(p_robot);
p = zeros(3,n);

for i = 1:n
    T = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
    for N = 1:i-2
        T = T * expm(skew4(S(:,N))*theta(N));
    end
    p_1 = T * [p_robot(:,i);1];
    p(:,i) = p_1(1:3,1);
end
end