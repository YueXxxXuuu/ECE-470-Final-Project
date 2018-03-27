%Author: Won Dong Shin
%Email: shin123@illinois.edu

function [rtn1, rtn2] = forwardKinematics(a,d,alp,th)
%% Instruction
%Parameters==================================================================================
%Inputs:
%   a: DH parameter a in vector form
%   d: DH parameter d in vector form
%   alp: DH parameter alpha in radian in vector form
%   th: DH parameter theta in radian in vector form
%Output:
%   rtn: 4X4 homogeneous transformation matrices 
%        rtn(:,:,1) = H01, rtn(:,:,2) = H02, .......
%============================================================================================

%% Calculation
    dataSize = max(size(a));

    %Initialize the matrix
    matrix = sym(zeros(4,4,dataSize));
    matrix_end = sym(zeros(4,4,dataSize));
    for i = 1: dataSize + 1
        matrix(1:4,1:4,i) = eye(4);
        matrix_end(1:4,1:4,i) = eye(4);
    end

    for i = 1:dataSize
        matrix_end(:,:,i + 1) =  matrix_end(:,:,i)*DHmatrix(a(i),d(i),alp(i),th(i));
    end
    
    for i = 1:dataSize
        matrix(:,:,i + 1) =  matrix_end(:,:,i)*DHmatrix(a(i)/2,d(i)/2,alp(i),th(i));
    end

    rtn1 = matrix(:,:,2:dataSize + 1);
    rtn2 = matrix_end(:,:,2:dataSize + 1);

end