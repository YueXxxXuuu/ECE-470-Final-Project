function  c = collision_check(p_p, p, r_1, r_2)
    [~,n] =size(p_p);
    c = 0;
    for i = 1:n
         if collision(p_p(:,i), p, r_1, r_2) == 1
                    c = 2;
    end
end