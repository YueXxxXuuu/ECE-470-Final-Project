function [R] = getR(angles)

    R = rot3x3('x', angles(1))*rot3x3('y', angles(2))*rot3x3('z', angles(3));

end