function rtn = collision(p_1, p_2, r_1, r_2)
    distance = sqrt((p_2(1)-p_1(1))*(p_2(1)-p_1(1)) + (p_2(2)-p_1(2))*(p_2(2)-p_1(2)) + (p_2(3)-p_1(3))*(p_2(3)-p_1(3)));
    r = r_1+r_2;
    
    if r>distance
        rtn  = 1;
    else 
        rtn  = 0;
    end

end