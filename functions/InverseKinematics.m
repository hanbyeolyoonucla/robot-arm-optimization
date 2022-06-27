function [q_iteration, error] = InverseKinematics(T_desired,Screw,M_EF,q_init,n_joint)
    
    % Stepsize
    k = 0.3;
    
    % Tolerance for stopping iteration
    tolerance = 0.0001;%¿ø·¡´Â 0.0001 YHB
    
    % Initial SE(3)
    q_iteration = q_init ;
    T = ScrewstoSE3(Screw, q_iteration) * M_EF;
    
    iteration = 1;
    
    while norm(T-T_desired) > tolerance
        
        error(iteration) = norm(T-T_desired);
        
        % Derive q after one iteration from before q
        K = logm(T_desired/T);
        V(1:3,1) = [K(3,2);K(1,3);K(2,1)];
        V(4:6,1) = K(1:3,4);
        delta_q =  k * pinv(SJacobian(Screw,q_iteration,n_joint))* V ;
        q_iteration = q_iteration + delta_q;
        
        % SE(3) after iteration
        T = ScrewstoSE3(Screw , q_iteration) * M_EF;
        
        iteration = iteration + 1;
        
    end
    
    % Simplify results' value to [0,2*pi)
    q_iteration = mod(q_iteration,2*pi);
    
end
