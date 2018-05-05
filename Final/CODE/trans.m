%Author: Won Dong Shin
%Email: shin123@illinois.edu

function [rtn] = trans(axis,d)
%% Instruction
%Parameters====================================================================================
%Inputs:
%   axis: Rotation axis (Options: 'x', 'y', 'z')(Not case sensitive)
%   d: Distance we want to translate along the given axis
%Output:
%   rtn: 4X4 Homogenous transformation(translation) matrix
%==============================================================================================

%Note================================================================= 
%  Make sure you put single quotation mark arround the variable 'axis' 
%  when you call this function
%===================================================================== 

%Example==================================
% >> trans('x',1)
% 
% ans =
% 
%     1.0000        0        0    1.0000
%          0   1.0000   0.0000    0.0000
%          0   0.0000   1.0000    0.0000
%          0        0        0    1.0000
%=========================================

%% Calculation
    %Initialization
    rtn = sym(eye(4));
    
    if(axis == 'x' || axis == 'X')
        rtn(1:3,4) = [d;0;0]; 
    end
    
    if(axis == 'y' || axis == 'Y')
        rtn(1:3,4) = [0;d;0]; 
    end
    
    if(axis == 'z' || axis == 'Z')
        rtn(1:3,4) = [0;0;d]; 
    end
    
end