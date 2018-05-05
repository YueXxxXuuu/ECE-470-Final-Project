function  s = path_planning(T_base_in_world, p_obstacle, r_obstacle, theta_start, theta_goal)
    [Size,~] = size(theta_start);
    T_forward(1).th = theta_start;
    T_forward(1).parent = 666666;
    T_forward(1).child = 666666;
    n = 1;
    T_backward(1).th = theta_goal;
    T_backward(1).parent = 666666;
    T_backward(1).child = 666666;
    m = 1;

    for count = 1:100
        c = 1;
        while c == 1
            theta = [rand()*2*pi;rand()*200/180*pi+70/180*pi;rand()*250/180*pi+40/180*pi;rand()*2*pi;rand()*2*pi;rand()*2*pi];
            for i = 1:Size
               theta(i,1) = theta(i,1)- pi;
            end
            c = collision_point(T_base_in_world, p_obstacle, r_obstacle, theta);
        end
        
        min = 100;
        idx = 0;
        for i = 1:n
            d = distance(theta, T_forward(i).th);
            if d < min
               min = d;
               idx = i;
            end
        end
        
        collide_forward = collision_line(T_base_in_world, p_obstacle, r_obstacle,  T_forward(idx).th, theta);
        if collide_forward == 0
            T_forward(n + 1).th = theta;
            T_forward(n + 1).parent = idx;
            T_forward(idx).child = n + 1;
            n = n + 1;
        end
        
        min = 100;
        idx = 0;
        for i = 1:m
            d = distance(theta, T_backward(i).th);
            if d < min
               min = d;
               idx = i;
            end
        end
        collide_backward = collision_line(T_base_in_world, p_obstacle, r_obstacle,  T_backward(idx).th, theta);
        if collide_backward == 0
            T_backward(m + 1).th = theta;
            T_backward(m + 1).parent = idx;
            T_backward(idx).child = m + 1;
            m = m + 1;
        end
        
        if T_backward(m).th == T_forward(n).th
            x = 1;
            i = 1;
            while i ~= n
                s(:,x) = T_forward(i).th;
                i = T_forward(i).child;
                x = x + 1;
            end
            s(:,x) = T_forward(i).th;
            x = x + 1;
            
            j = T_backward(m).parent;
            while j ~= 1
                s(:,x) = T_backward(j).th;
                j = T_backward(j).parent;
                x = x + 1;
            end
            s(:,x) = T_backward(j).th;
            x = x + 1;
            return;
            
        end
    end
       s = 0;
end