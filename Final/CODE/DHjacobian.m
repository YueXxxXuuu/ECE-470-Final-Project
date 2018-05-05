function rtn = DHjacobian(theta)
    theta = vpa(theta,4);
    dataSize = 6;
    H = forward(theta);
    H = vpa(H,4);
    %p is location of origin in base frame
    p = zeros(3,dataSize + 1);
    %z is first 3 rows of third column of H matrix
    z = zeros(3,dataSize + 1);
    z(1:3,1) = [0;0;1];
    rtn = zeros(6,dataSize);
    
    for i=1:dataSize
        p(1:3,i + 1) = H(1:3,4,i);
        z(1:3,i + 1) = H(1:3,3,i);
    end    
    
    for i = 1:dataSize
            rtn(1:3,i) = cross(z(1:3,i),(p(1:3,dataSize+1)-p(1:3,i)));
            rtn(4:6,i) = z(1:3,i);
    end
    
end