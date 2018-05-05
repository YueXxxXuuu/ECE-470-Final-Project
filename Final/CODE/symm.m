function [rtn] = symm(V)
    a = V(1,1);
    b = V(2,1);
    c = V(3,1);
    
    rtn = [0,-c,b;c,0,-a;-b,a,0];
end