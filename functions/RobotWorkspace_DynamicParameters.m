function [DD, QQ, g_x, x, dxdt, dJdt] = RobotWorkspace_DynamicParameters(S, M, MM, CC, gg, q, dqdt, n_torquejoint)
    
    n_joint = 6;
    
    % Classic Jacobian
    [J_a,garbage,garbage2] = AJacobian(S, q, M, n_torquejoint);
%     J_a = J_a(4:6,1:n_torquejoint);
   
    % D matrix
    DD = pinv(J_a') * MM * pinv(J_a) ;

    % Q matrix
    step = 0.01;
    
    dJdt = zeros(6,n_torquejoint);
    for i=1:n_torquejoint
        q_dt = zeros(n_torquejoint,1);
        q_dt(i,1) = step;
        [J_tplusdt,garbage,garbage2 ]= AJacobian(S, q+q_dt, M, n_torquejoint);
%         J_tplusdt = J_tplusdt(4:6,1:n_torquejoint);
        dJdt = dJdt +(J_tplusdt - J_a)/step * dqdt(i,1) ;
    end
    
    dinvJdt = - pinv(J_a) * dJdt * pinv(J_a);

    QQ = pinv(J_a') * (MM * dinvJdt + CC * pinv(J_a));

    % g_x matrix
    g_x = pinv(J_a')*gg;
    
    % position and velocity of ef location
    T_5 = ScrewstoSE3(S(:,1:n_torquejoint), q) * M; 
    x_temp = T_5(1:3,4);
    euler_temp = rotm2eul(T_5(1:3,1:3));
    x = [euler_temp(3);euler_temp(2);euler_temp(1);x_temp ];
    dxdt = J_a * dqdt;
   
end