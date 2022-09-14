function [q] = rotm2eulXYZ(R)
    if R(1,3) == 1
        alpha = 0;
        beta = pi/2;
        gamma = atan2(R(2,1),R(2,2));
    elseif R(1,1) == -1
            alpha = 0;
            beta = -pi/2;
            gamma = atan2(R(2,1),R(2,2));
    else
        alpha = atan2(-R(2,3),R(3,3));
        gamma = atan2(-R(1,2),R(1,1));
        beta = atan2(R(1,3),sqrt(R(1,1)^2+R(1,2)^2));
    end
    q = [alpha;beta;gamma];
end