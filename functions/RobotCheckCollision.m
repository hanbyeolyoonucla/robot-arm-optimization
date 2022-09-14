function collisionStatus = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q,n_joint,n_link, L1,L2,L3,L4,L5,Ltool,Ltool1)

Radius = [45 45 25 35 25 30 10]/1000;
LEE = 40/1000;
if checkCollisionOn == 1
    Ltool2 = sqrt(Ltool^2+Ltool1^2);
    alphatool = atan2(Ltool1,Ltool);
    L = [L1, L2, L3, L4, L5, LEE,Ltool2];
    
    % Cylinder for checking collision
    CYL_T = cell(n_link,1);
    CYL = cell(n_link,1);
    CYL_q = cell(n_link,1);
    CYL_q{1} = [0;0;L1/2];
    CYL_q{2} = [0;0;L1+L2/2];
    CYL_q{3} = [0;0;L1+L2+L3/2];
    CYL_q{4} = [L4/2;0;L1+L2+L3];
    CYL_q{5} = [L4;0;L1+L2+L3-L5/2];
    CYL_q{6} = [L4;0;L1+L2+L3-L5-LEE/2];
    CYL_q{7} = [L4+Ltool/2*tan(alphatool);0;L1+L2+L3-L5-Ltool/2];
    for ii = 1:n_link
        CYL_T{ii} = [eye(3) CYL_q{ii}; zeros(1,3) 1];
    end
    CYL_T{4}(1:3,1:3) = rot('y',pi/2);
    CYL_T{7}(1:3,1:3) = rot('y',alphatool);
    
    % Transform the cylinder given joint angles
    POE = eye(4);
    for i=1:n_link
        if i <= n_joint
            POE = POE*OneScrewtoSE3(S(:,i),q(i,1));
        end
        CYL_T{i} = POE*CYL_T{i};
        CYL{i} = collisionCylinder(Radius(i), L(i));
        CYL{i}.Pose = CYL_T{i};
        if drawCylinderOn == 1
            show(CYL{i})
        end
    end
    
    % Check collision
    for i = 1:4
        collisionStatus = checkCollision(CYL{7},CYL{i});
        if collisionStatus == 1
            break
%         else
%             collisionStatus = checkCollision(CYL{6},CYL{i});
%         end
%         if collisionStatus == 1
%             break
        end
    end
    
else
    collisionStatus = 0;
end

end