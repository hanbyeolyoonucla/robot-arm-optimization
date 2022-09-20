clear
close all
clc

addpath('functions');
tic

checkCollisionOn = 1;
drawCylinderOn = 0;
occusalCutOn = 1;
axialCutOn = 1;
maxillaOn = 0;
mandibleOn = 1;
n_space = 3;
n_angle = 5;

% Robot's DOF
n_joint = 6;
% discretization of search space
cost = zeros(10,n_space,n_space,n_space);
% fixed parameters
L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
L5 = 0.070;
Ltool = 0.140;
Ltool1 = 0.091; %tool offet
alphatool = -77*pi/180;
ang_mouthOpen = 20*pi/180;


% 0. Forward Kinematics

% Joint positions' SE(3) when qi = 0
M = cell(n_joint,1);
joint_q = cell(n_joint,1);
joint_q{1} = [0;0;0];
joint_q{2} = [0;0;L1];
joint_q{3} = [0;0;L1+L2];
joint_q{4} = [L4/2;0;L1+L2+L3];
joint_q{5} = [L4;0;L1+L2+L3];
joint_q{6} = [L4;0;L1+L2+L3-L5];
for i=1:6
    M{i} = [eye(3) joint_q{i};zeros(1,3) 1];
end

% Joint screws when qi = 0
S = zeros(6,6);
joint_w = cell(n_joint,1);
joint_w{1} = [0;0;1];
joint_w{2} = [0;1;0];
joint_w{3} = [0;1;0];
joint_w{4} = [1;0;0];
joint_w{5} = [0;1;0];
joint_w{6} = [0;0;-1];
for i=1:6
    S(:,i) = [joint_w{i} ; - cross(joint_w{i},joint_q{i},1)];
end

% EF frame's SE(3) when qi = 0
EF_q = [L4; 0; L1+L2+L3-L5-Ltool];
EF_w = rot('y',alphatool)*[0;0;1];
M_EF = [rot('y',alphatool) EF_q;zeros(1,3) 1];

% Joint shape information for drawing robot
JointDiameter = 20/1000;
JointLength = 24/1000;

% Bruth Force Approach
alpha_space = linspace(0,pi/2,10);
x_space = linspace(-L2-sqrt(L3^2+L4^2),L2+sqrt(L3^2+L4^2),n_space);
y_space = linspace(0.1,L2+sqrt(L3^2+L4^2),n_space);
z_space = linspace(L1-L2-sqrt(L3^2+L4^2)-L5-Ltool,L1+L2+sqrt(L3^2+L4^2)-L5-Ltool,n_space);
for i_alpha = 1:10
    alpha = alpha_space(i_alpha);
    for i_x = 1:n_space
        x_cube = x_space(i_x);
        for i_y = 1:n_space
            y_cube = y_space(i_y);
            for i_z = 1:n_space
                z_cube = z_space(i_z);
                
                % 1. Define Work Space
                R_SJ = rot('z',pi)*rot('y',alpha);
                p_SJ = [x_cube;y_cube;z_cube];
                T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
                [T_ST,p_ST] = DefineWorkSpace(maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);
                
                % 2. Analytic Inverse Kinematics / Check Collision / Compute ISO
                [n_teeth,n_discrete] = size(T_ST);
                q_IK = zeros([n_joint,size(T_ST)]);
                eflag = zeros(size(T_ST));
                ISO_ang_sum = 0;
                ISO_ang = zeros(size(T_ST));
                ISO_lin_sum = 0;
                ISO_lin = zeros(size(T_ST));
                for ii = 1:n_teeth
                    for jj = 1:n_discrete
                        [q_IK(:,ii,jj),eflag(ii,jj),~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,S,M_EF,T_ST{ii,jj});
                        % if no IK exits, break
                        if eflag(ii,jj) == 0
                            break
                        else % if IK exists
                            collisionStatus = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q_IK(:,ii,jj),n_joint,7,...
                                L1,L2,L3,L4,L5,Ltool,Ltool1);
                            if collisionStatus == 0
                                [J, ~, ~] = AJacobian(S,q_IK(:,ii,jj),M_EF,n_joint);
                                J_v = J(4:6,:);
                                J_w = J(1:3,:);
                                SingularValue_lin = svd(J_v);
                                SingularValue_ang = svd(J_w);
                                ISO_lin(ii,jj) = SingularValue_lin(end)/SingularValue_lin(1);
                                ISO_ang(ii,jj) = SingularValue_ang(end)/SingularValue_ang(1);
                                ISO_lin_sum = ISO_lin_sum + ISO_lin(ii,jj);
                                ISO_ang_sum = ISO_ang_sum + ISO_ang(ii,jj);
                            else
                                break
                            end
                        end
                    end
                    if eflag(ii,jj) == 0 || collisionStatus == 1
                        break
                    end
                end
                ISO_lin_min = min(ISO_lin,[],'all');
                ISO_ang_min = min(ISO_ang,[],'all');
                
                % display for checking iteration status
                cost(i_alpha,i_x,i_y,i_z) = ISO_lin_sum*ISO_ang_sum*ISO_lin_min*ISO_ang_min...
                    /(L1^3+L2^3+(sqrt(L3^2+L4^2))^3+L5^3);
                disp([i_alpha,i_x,i_y,i_z])
                disp(cost(i_alpha,i_x,i_y,i_z))
            end
        end
    end
end

toc

% result summary
[val, idx] = max(cost,[],'all','linear');
[idx1, idx2, idx3,idx4] = ind2sub( size( cost ), idx );
alpha = alpha_space(idx1);
x_cube = x_space(idx2);
y_cube = y_space(idx3);
z_cube = z_space(idx4);

%% Visualization of Optimal Solution

% Define Optimal WS
R_SJ = rot('z',pi)*rot('y',alpha);
p_SJ = [x_cube;y_cube;z_cube];
p_PR = -R_SJ'*p_SJ;
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST,p_ST] = DefineWorkSpace(maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% Solve IK for Optimal WS
q_IK = zeros([n_joint,size(T_ST)]);
for ii = 1:n_teeth
    for jj = 1:n_discrete
        [q_IK(:,ii,jj),~,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,S,M_EF,T_ST{ii,jj});
    end
end
q_Meca = q_IK;
q_Meca(5,:,:) = q_Meca(5,:,:) + pi/2;

% T visualization : rotate the robot base by alpha for visualization
Rvis = rot('y',alpha);
Tvis = [Rvis zeros(3,1); zeros(1,3) 1];
Ad_Tvis = AdjointT(Tvis);
Mvis = cell(n_joint,1);
temp = reshape(Tvis*[M{:}],[4,4,6]);
for i = 1:n_joint
    Mvis{i} = temp(:,:,i);
end
Svis = Ad_Tvis*S;
EF_wvis = Rvis*EF_w;
M_EFvis = Tvis*M_EF;

% Draw WS to stand upright for visualziation
R_SJ = rot('z',pi);
p_SJ = Rvis*[x_cube;y_cube;z_cube];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST,p_ST] = DefineWorkSpace(maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

%% Draw robot in zero position tilted by alpha
figure(1)
q = zeros(6,1);
T_EF = Draw_Robot_Meca(Svis,Mvis,q,EF_wvis,M_EFvis,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(Rvis),'FrameSize',0.05)
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
plotTransforms(p_ST(:,11)',rotm2quat(T_ST{11,1}(1:3,1:3)),'FrameSize',0.02)
plotTransforms(p_ST(:,11)',rotm2quat(T_ST{11,2}(1:3,1:3)),'FrameSize',0.02)
plotTransforms(p_ST(:,11)',rotm2quat(T_ST{11,3}(1:3,1:3)),'FrameSize',0.02)
xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on

%% Draw robot in random sample IK
figure(2)
T_EF = Draw_Robot_Meca(Svis,Mvis,q_IK(:,16,4),EF_wvis,M_EFvis,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(Rvis),'FrameSize',0.05)
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on

%% Draw all possible position of PRF
% all possible position of PRF
costOptAlpha = cost(idx1,:,:,:);
ind = find(costOptAlpha);
[i1, i2, i3, i4] = ind2sub(size(costOptAlpha), ind);
costNZ = zeros(length(i1),1);
for ii = 1:length(i1)
    costNZ(ii) = costOptAlpha(i1(ii),i2(ii),i3(ii),i4(ii));
end
posPRF = [x_space(i2)' y_space(i3)' z_space(i4)']';
posPRF = Rvis*posPRF;
figure(2)
scatter3(posPRF(1,:),posPRF(2,:),posPRF(3,:),5, costNZ)
colormap(jet);
colorbar;

%% Draw cylinders for check collision
drawCylinderOn = 1;
figure(3)
T_EF = Draw_Robot_Meca(S,M,q_IK(:,1,2),EF_w,M_EF,n_joint,JointDiameter,JointLength);
RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q_IK(:,1,2),n_joint,7,...
                                L1,L2,L3,L4,L5,Ltool,Ltool1);
hold on
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(Rvis),'FrameSize',0.05)
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on

%% save result
save('data/MecaBasePose_220914_mandible_occusal_axial','n_space','n_angle','L1','L2','L3','L4','L5',...
    'alpha','r_cube','y_cube','z_cube','alpha_space','r_space','y_space','z_space','posPRF','costNZ','p_PR','q_Meca','cost');