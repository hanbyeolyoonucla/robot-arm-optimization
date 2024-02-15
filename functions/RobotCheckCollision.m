function collisionStatus = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q,n_joint,n_link, L1,L2,L3,L4,L5,Ltool,Ltool1, T_SJ)

Radius = [45 45 25 35 25 30 10]/1000;
LEE = 40/1000;
if checkCollisionOn == 1
    Ltool2 = sqrt(Ltool^2+Ltool1^2);
    alphatool = atan2(Ltool1,Ltool);
    L = [L1, L2, L3, L4, L5, LEE,Ltool2];
    
    % head center / body center
    head_p = T_SJ * [0;0;-0.1;1];
    head_p = head_p(1:3);
    body_p = T_SJ * [-0.4;0;-0.1;1];
    body_p = body_p(1:3);
    
    % Cylinder for checking collision
    CYL_T = cell(n_link,1);
    CYL = cell(n_link,1);
    CYL_q = cell(n_link,1);
    CYL_q{1} = [0;0;L1/2];
    CYL_q{2} = [0;0;L1+L2/2];
    CYL_q{3} = [0;0;L1+L2+L3/2];
    CYL_q{4} = [L4/2;0;L1+L2+L3];
    CYL_q{5} = [L4;0;L1+L2+L3-L5/2];
    CYL_q{6} = [L4;0;L1+L2+L3-L5-LEE/2]; % tool1
    CYL_q{7} = [L4+Ltool/2*tan(alphatool);0;L1+L2+L3-L5-Ltool/2]; % tool2
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
            show(CYL{i}); hold on;
        end
    end
    
    % head and body collision object
    head = collisionCylinder(0.1,0.2);
    body = collisionBox(0.6,0.6,0.2);
    head.Pose = [T_SJ(1:3,1:3) head_p; zeros(1,3) 1];
    body.Pose = [T_SJ(1:3,1:3) body_p; zeros(1,3) 1];
    if drawCylinderOn == 1
            show(head)
            show(body)
    end
    
%     % Check collision between links to ee
%     for i = 1:4
%         collisionStatus = checkCollision(CYL{7},CYL{i});
%         if collisionStatus == 1
%             break
%         else
%             collisionStatus = checkCollision(CYL{6},CYL{i});
%         end
%         if collisionStatus == 1
%             break
%         end
%     end
    
    % Check collision between links to patient
    for i = 1:4
        collisionStatus = checkCollision(head,CYL{i});
        if collisionStatus == 1
            break
        else
            collisionStatus = checkCollision(body,CYL{i});
        end
        if collisionStatus == 1
            break
        end
    end
    
else
    collisionStatus = 0;
end

end


%% test code
% clc; close all; clear;
% addpath('./functions')
% R_SJ = rot('z',pi);
% p_SJ = [0.2; 0; 0.1];
% T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
% 
% % 1. Input Varialbes
% L1 = 0.135;
% L2 = 0.135;
% L3 = 0.038;
% L4 = 0.120;
% L5 = 0.070;
% x_EH = 0; z_EH = 0.1; phi_EH = pi/2;
% alphatool = -90*pi/180; 
% 
% % 2. Forward Kinematics
% % Robot's DOF
% n_joint = 6;
% % Joint positions' SE(3) when qi = 0
% M = cell(n_joint,1);
% joint_q = cell(n_joint,1);
% joint_q{1} = [0;0;0];
% joint_q{2} = [0;0;L1];
% joint_q{3} = [0;0;L1+L2];
% joint_q{4} = [L4/2;0;L1+L2+L3];
% joint_q{5} = [L4;0;L1+L2+L3];
% joint_q{6} = [L4;0;L1+L2+L3-L5];
% for i=1:6
%     M{i} = [eye(3) joint_q{i};zeros(1,3) 1];
% end
% % Joint screws when qi = 0
% S = zeros(6,6);
% joint_w = cell(n_joint,1);
% joint_w{1} = [0;0;1];
% joint_w{2} = [0;1;0];
% joint_w{3} = [0;1;0];
% joint_w{4} = [1;0;0];
% joint_w{5} = [0;1;0];
% joint_w{6} = [0;0;-1];
% for i=1:6
%     S(:,i) = [joint_w{i} ; - cross(joint_w{i},joint_q{i},1)];
% end
% % EF frame's SE(3) when qi = 0
% EF_q = [L4 + x_EH; 0; L1+L2+L3-L5-z_EH];
% M_EF = [rot('y',alphatool+phi_EH) EF_q;zeros(1,3) 1];
% 
% checkCollisionOn = 1; drawCylinderOn = 1;
% q = zeros(6,1); n_link = 7; Ltool = 0.144; Ltool1 = 0.091; %tool offet
% collisionStatus = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q,n_joint,n_link, L1,L2,L3,L4,L5,Ltool,Ltool1, T_SJ)