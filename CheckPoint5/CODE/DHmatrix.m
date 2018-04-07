%Author: Won Dong Shin
%Email: shin123@illinois.edu

function [rtn] = DHmatrix(a,d,alp,th)
%% Instruction
%Parameters==================================================================================
%Inputs:
%   a: DH parameter a
%   d: DH parameter d
%   alp: DH parameter alpha in radian
%   th: DH parameter theta in radian
%Output:
%   rtn: 4X4 DH matrix
%============================================================================================

%% Calculation
    rtn =  rot('z',th)*trans('z',d)*trans('x',a)*rot('x',alp);

end