function [q, eflag, p_SW_log] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST)

    % find wrist position
    offsetAngle = atan2(L4,L3);
    L34 = sqrt(L3^2+L4^2);
    p_TW = rot('y',-alphatool-pi/2)*[L5+Ltool;0;0];
    p_SW = T_ST*[p_TW;1]; % 4x1 vector
    p_SW_log = p_SW(1:3); % 3x1 position vector
    p_SW = p_SW_log - [0;0;L1]; % subtract L1 for IK

    % solve IK
    q = zeros(6,1);
    r = norm(p_SW);
    if r > L2 + L34 || L2 > r + L34 || L34 > L2 + r % check if IK exists
        eflag = 0;
    else
        % positional IK
        q(1) = atan2(p_SW(2),p_SW(1));
        D = (r^2 - L2^2 - L34^2) / (2*L2*L34);
        q(3) = atan2(sqrt(1-D^2),D) - offsetAngle;
        q(2) = pi/2 - atan2(p_SW(3),sqrt(p_SW(1)^2+p_SW(2)^2)) - atan2(L34*sin(q(3)+offsetAngle),L2+L34*cos(q(3)+offsetAngle));

        % angular IK
        M_EF_inv = [M_EF(1:3,1:3)' -M_EF(1:3,1:3)'*M_EF(1:3,4); zeros(1,3) 1];
        T = ScrewstoSE3(S(:,1:3),q(1:3));
        T_inv = [T(1:3,1:3)' -T(1:3,1:3)'*T(1:3,4); zeros(1,3) 1];
        T_anguler = T_inv*T_ST*M_EF_inv;
        R = T_anguler(1:3,1:3);
        q(4:6) = rotm2eulXYZ(R);
        q(6) = -q(6);
        
        % check joint limit
        if CheckMecaJointLimit(n_joint,q) ~= 0
            eflag = 0;
        else
            eflag = 1;
        end
    end

end