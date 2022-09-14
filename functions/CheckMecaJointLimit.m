function jointViolated = CheckMecaJointLimit(n_joint,q)

% Meca joint limit
jointLim = [-175 175;-70 90;-135 70;-170 170;-115 115;-3600 3600];
jointLim(1,:) = jointLim(1,:) - 90;
jointLim(5,:) = jointLim(5,:) - 90;
jointLim = jointLim*pi/180;
jointViolated = 0;
for ii = 1:n_joint
    if q(ii) < jointLim(ii,1) || q(ii) > jointLim(ii,2)
        jointViolated = ii;
    end
end