function [rtn] = revolute(a,q)
   rtn = [a; -symm(a)*q];
end