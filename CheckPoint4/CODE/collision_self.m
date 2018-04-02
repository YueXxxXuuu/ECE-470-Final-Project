function  c = collision_self(joint_angles)
    c = 0;
    angles = zeros(1,6);
    for i= 1:6
        angles(i) = joint_angles(i) + pi;
    end
    for i= 1:6
     while  angles(i) < 0
        angles(i) = angles(i) + 2*pi;
     end
     while  angles(i) > (2 * pi)
         angles(i) = angles(i) - 2*pi;
     end
    end
    for i= 1:6
        angles(i) = angles(i) * 180/pi;
    end

    if angles(2) < 47
        c =1;
    end
    if angles(2) > 313
        c =1;
    end
    if angles(3) < 19
        c =1;
    end
    if angles(3) > 341
        c =1;
    end
    
end