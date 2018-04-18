function u = qp_gen(x_act)
    x_des = 0;
    kp = 5;
    angle_limit = 20;

    H = [100, 0; 0, 1];
    h = [0, 0];
    
    % stability constraint
    LgV = x_act - x_des;
    LfV = -kp * (x_act - x_des).^2;

    % safety constraint

    LgB = - 2. * x_act;
    LfB = - (angle_limit - x_act^2);

    % net constraints
    A = [-1, LgV;0,-LgB];
    b = [LfV;-LfB];
    options = optimoptions('quadprog','display','off');
%     options = [];
            
    u = quadprog(H,h,A,b,[],[],[],[],[],options);
%     u = - x_in;
end