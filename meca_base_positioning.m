clear
close all
clc
%% Optimization Parameters

addpath('functions');
tic

checkCollisionOn = 1;
drawCylinderOn = 0;
occusalCutOn = 1;
axialCutOn = 1;
maxillaOn = 1;
mandibleOn = 1;
halfOn = 1;
n_space = 30;
n_angle = 5;

% Robot's DOF
n_joint = 6;
% fixed parameters
L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
L5 = 0.070;
Ltool = 0.160;
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

%% Bruth Force Approach

% Search Space
alpha_space = linspace(-pi/6,pi/3,10);
beta_space = linspace(-50*pi/180,40*pi/180,10);
% gamma_space = linspace(0,pi,3);
x_space = linspace(-L2-sqrt(L3^2+L4^2),L2+sqrt(L3^2+L4^2),n_space);
y_space = linspace(-L2-sqrt(L3^2+L4^2),L2+sqrt(L3^2+L4^2),n_space);
z_space = linspace(L1-L2-sqrt(L3^2+L4^2),L1+L2+sqrt(L3^2+L4^2)+L5+Ltool,n_space);

score = zeros(10,10,n_space,n_space,n_space);
unreachableConfig = zeros(10,10,n_space,n_space,n_space);
collisionConfig = zeros(10,10,n_space,n_space,n_space);

gamma = pi/2;
for i_beta = 1:10
    beta = beta_space(i_beta);
    for i_alpha = 1:10
        alpha = alpha_space(i_alpha);
        for i_x = 1:n_space
            x_cube = x_space(i_x);
            for i_y = 1:n_space
                y_cube = y_space(i_y);
                for i_z = 1:n_space
                    z_cube = z_space(i_z);
                    
                    % 1. Define Work Space
                    R_SJ = rot('z',-gamma)*rot('y',-alpha)*rot('x',beta)*rot('z',pi);
                    p_SJ = [x_cube;y_cube;z_cube];
                    T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
                    [T_ST,~] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);
                    
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
                            [q_IK(:,ii,jj),eflag(ii,jj),~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST{ii,jj});
                            % if no IK exits, break
                            if eflag(ii,jj) == 0
                                unreachableConfig(i_beta,i_alpha,i_x,i_y,i_z) = unreachableConfig(i_beta,i_alpha,i_x,i_y,i_z)-1;
                                %                                       break
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
                                    collisionConfig(i_beta,i_alpha,i_x,i_y,i_z) = collisionConfig(i_beta,i_alpha,i_x,i_y,i_z) - 1;
                                    %                                           break
                                end
                            end
                        end
                        %                             if eflag(ii,jj) == 0 || collisionStatus == 1
                        %                                 break
                        %                             end
                    end
                    ISO_lin_min = min(ISO_lin,[],'all');
                    ISO_ang_min = min(ISO_ang,[],'all');
                    ISO_lin_avg = ISO_lin_sum/(n_teeth*n_discrete);
                    ISO_ang_avg = ISO_lin_sum/(n_teeth*n_discrete);
                    
                    % Stiffness Normalization
                    stiffNormalized = stiffnessScore(L1,L2,L3,L4,L5);
                    
                    % Manipulability
                    manipulability = ISO_lin_avg*ISO_ang_avg*ISO_lin_min*ISO_ang_min;
                    
                    % display for checking iteration status
                    score(i_beta,i_alpha,i_x,i_y,i_z) = manipulability*stiffNormalized;
                    disp([i_beta,i_alpha,i_x,i_y,i_z])
                    disp(score(i_beta,i_alpha,i_x,i_y,i_z))
                    %                         disp(unreachableConfig(i_gamma,i_beta,i_alpha,i_x,i_y,i_z))
                    %                         disp(collisionConfig(i_gamma,i_beta,i_alpha,i_x,i_y,i_z))
                end
            end
        end
    end
end
toc

%% result summary

% x,y,z with max score for each pose
x = zeros(10,10,5);
for jj = 1:10
    for kk = 1:10
        [val, idx] = max(score(jj,kk,:,:,:),[],'all','linear');
        [~,~,idx4,idx5,idx6] = ind2sub( size( score(jj,kk,:,:,:) ), idx );
        x(jj,kk,:) = [beta_space(jj), alpha_space(kk), x_space(idx4), y_space(idx5), z_space(idx6)];
    end
end

% x,y,z,alpha, beta for max score for entire pose
[optimal_score, idx] = max(score,[],'all','linear');
[idx2,idx3,idx4,idx5,idx6] = ind2sub( size( score ), idx );
optimal_x = [ beta_space(idx2), alpha_space(idx3), x_space(idx4), y_space(idx5), z_space(idx6)];

% Reachable volume of PRF origin
volume = zeros(10,10);
for jj = 1:10
    for kk = 1:10
        ind = find(score(jj,kk,:,:,:));
        [~,~,i4,i5,i6] = ind2sub(size(score(jj,kk,:,:,:)), ind);
        posPRF = [x_space(i4)' y_space(i5)' z_space(i6)']';
        volume(jj,kk) = length(posPRF)*(x_space(2)-x_space(1))*(y_space(2)-y_space(1))*(z_space(2)-z_space(1));
    end
end

%% Plot: angle vs volume

[alpha, beta] = meshgrid(alpha_space, beta_space);
alpha = alpha*180/pi;
beta = beta*180/pi;
figure(1)
mesh(alpha,beta,squeeze(volume(:,:)), 'FaceColor', 'flat')
hold on; grid on;
view(3)
% xlim([-30 60])
% ylim([-30 60])
zlim([0 1.5e-3])
zlabel('Volume of Reachable P.R.F. [m^3]')
xlabel('\alpha [degree]')
ylabel('\beta [degree]')

title('\alpha \beta vs Volume')

%% result plot

% Specify the pose which we want to plot
gamma_space = pi/2;
ii=1;
jj=1; %beta
kk=8; %alpha

% T visualization : rotate the robot base for visualization
Rvis = rot('x',-beta_space(jj))*rot('y',alpha_space(kk))*rot('z',gamma_space(ii));
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
p_SJ = Rvis*[x(jj,kk,3);x(jj,kk,4);x(jj,kk,5)];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST,p_ST] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% Draw robot in random sample IK
figure(2)
title(['\beta = ',num2str(x(jj,kk,1)*180/pi), char(176), '\alpha = ',num2str(x(jj,kk,2)*180/pi), char(176)])
q = zeros(6,1);
q(5) = q(5) - pi/2;
T_EF = Draw_Robot_Meca(Svis,Mvis,q,EF_wvis,M_EFvis,Ltool1,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(Rvis),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.05)
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
xlabel('x [m]','FontSize',10);
ylabel('y [m]','FontSize',10);
zlabel('z [m]','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on

% all possible position of PRF
% costOptAlpha = score(idx1,:,:,:);
% ind = find(costOptAlpha);
ind = find(score(jj,kk,:,:,:));
[i2, i3, i4,i5,i6] = ind2sub(size(score(jj,kk,:,:,:)), ind);
costNZ = zeros(length(i4),1);
for ll = 1:length(i4)
    costNZ(ll) = score(jj,kk,i4(ll),i5(ll),i6(ll));
end
posPRF = [x_space(i4)' y_space(i5)' z_space(i6)']';
posPRF = Rvis*posPRF;
%             subplot(10,10,10*(jj-1)+kk)
scatter3(posPRF(1,:),posPRF(2,:),posPRF(3,:),5, costNZ,'filled')
colormap(jet);
colorbar;
hold on

% Find center of point clouds
[~, idx] = min(vecnorm(posPRF - mean(posPRF,2)));
centerPRF = posPRF(:,idx);
centerScore = score(jj,kk,i4(idx),i5(idx),i6(idx));
plot3(centerPRF(1),centerPRF(2),centerPRF(3),"pentagram",'MarkerSize',10)
% find max score point in point clouds
maxscorePRF = squeeze(x(jj,kk,3:5));
maxscorePRF = Rvis*maxscorePRF;
maxscore = max(costNZ);
plot3(maxscorePRF(1),maxscorePRF(2),maxscorePRF(3),"pentagram",'MarkerSize',10)


% Draw unreachable position of PRF
ind = find(unreachableConfig(jj,kk,:,:,:));
[i2, i3, i4,i5,i6] = ind2sub(size(unreachableConfig(jj,kk,:,:,:)), ind);
posPRF_unreachable = [x_space(i4)' y_space(i5)' z_space(i6)']';
posPRF_unreachable = Rvis*posPRF_unreachable;
%             subplot(10,10,jj+kk)
scatter3(posPRF_unreachable(1,:),posPRF_unreachable(2,:),posPRF_unreachable(3,:),'.','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerEdgeAlpha',0.2)

% Draw collision position of PRF
ind = find(collisionConfig(jj,kk,:,:,:));
[i2, i3, i4,i5,i6] = ind2sub(size(collisionConfig(jj,kk,:,:,:)), ind);
posPRF_collision = [x_space(i4)' y_space(i5)' z_space(i6)']';
posPRF_collision = Rvis*posPRF_collision;
scatter3(posPRF_collision(1,:),posPRF_collision(2,:),posPRF_collision(3,:),'.','MarkerEdgeAlpha',0.2,'MarkerEdgeColor',[0.8500 0.3250 0.0980])


%% save result
% mkdir data/221125
% saveas(figure(1),'data/221103/tolerance_angle_volume_plot')
% for ii = 1:10
%     saveas(figure(ii),['data/221103/tolerance_maxilla_',num2str((ii-1)*10),'.fig'])
% end
save('data/221204/mecaBasePosition_mandible','score','unreachableConfig','collisionConfig','beta_space','alpha_space','x_space','y_space','z_space')
% save('data/MecaBasePose_220914_mandible_occusal_axial','n_space','n_angle','L1','L2','L3','L4','L5',...
%     'alpha','r_cube','y_cube','z_cube','alpha_space','r_space','y_space','z_space','posPRF','costNZ','p_PR','q_Meca','cost');