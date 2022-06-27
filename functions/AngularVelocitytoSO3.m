function R = AngularVelocitytoSO3(w,q)
    
    % Rodrigue's Formula 
    cross = zeros(3,3);
    cross(1,2) = -w(3);
    cross(1,3) =  w(2);
    cross(2,3) =  -w(1);
    cross(2,1) =  -cross(1,2);
    cross(3,1) =  -cross(1,3);
    cross(3,2) =  -cross(2,3);
    
    R = eye(3)+sin(q)*cross+(1-cos(q))*cross^2;
    
end