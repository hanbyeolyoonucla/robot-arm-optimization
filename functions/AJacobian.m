function [AJacobian,EulerZYX,p] = AJacobian(Screw, q, M, n_torquejoint)
    
    n_joint = 6;
    q = [q;zeros(3,1)];
    
    J_s = SJacobian(Screw, q, n_joint);
    J_sw = J_s(1:3,:);
    J_sv = J_s(4:6,:);

    % End-effector at q configuration
    T = ScrewstoSE3(Screw(:,1:n_torquejoint), q) * M; 
    p = T(1:3,4);
    EulerZYX = rotm2eul(T(1:3,1:3));

    AJacobian_w = J_sw;   
    for i=1:n_torquejoint
        AJacobian_v(:,i) = J_sv(:,i) + cross(J_sw(:,i),p);
    end

    AJacobian = [AJacobian_w(:,1:n_torquejoint) ; AJacobian_v] ;

end

