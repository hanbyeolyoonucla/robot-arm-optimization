clear all
close all
clc

addpath('functions');
tic

% discretization of search space
n = 30;
cost = zeros(n,n,n,n,n);
L1_space = linspace(0.045,0.3,n);
L2_space = linspace(0.082,0.3,n);
L3_space = linspace(0.100,0.3,n);
L4_space = linspace(0.040,0.3,n);
L5_space = linspace(0.090,0.3,n);
% fixed parameters
% L1 = 0.135;
% L2 = 0.135;
% L3 = 0.038;
% L4 = 0.120;
% L5 = 0.070;
Ltool = 0.140;
Ltool1 = 0.091; %tool offet
alphatool = -167*pi/180;

alpha = 1.2217;
r_cube = 0.1499;
y_cube = -0.0630;
z_cube = 0.0779;

% Bruth Force Approach
for i_L1 = 1:n
    L1 = L1_space(i_L1);
    for i_L2 = 1:n
        L2 = L2_space(i_L2);
        for i_L3 = 1:n
            L3 = L3_space(i_L3);
            for i_L4 = 1:n
                L4 = L4_space(i_L4);
                for i_L5 = 1:n
                    L5 = L5_space(i_L5);
                    %                 alpha_space = linspace(0,pi/2,10);
                    %                 r_space = linspace(0.1,L2+sqrt(L3^2+L4^2),n);
                    %                 y_space = linspace(-L2-sqrt(L3^2+L4^2),L2+sqrt(L3^2+L4^2),n);
                    %                 z_space = linspace(L1-L2-sqrt(L3^2+L4^2)-L5-Ltool,L1+L2+sqrt(L3^2+L4^2)-L5-Ltool,n);
                    %                 for i_alpha = 1:10
                    %                     alpha = alpha_space(i_alpha);
                    %                 for i_r = 1:n
                    %                     r_cube = r_space(i_r);
                    %                     for i_y = 1:n
                    %                         y_cube = y_space(i_y);
                    %                     for i_z = 1:n
                    %                         z_cube = z_space(i_z);
                    
                    % 0. Forward Kinematics
                    
                    % Updating Frequency
                    dt = 0.001;
                    
                    % Robot's DOF
                    n_joint = 6;
                    
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
                    EF_w = rot('y',alphatool)*[1;0;0];
                    M_EF = [rot('y',alphatool) EF_q;zeros(1,3) 1];
                    
                    % Joint shape information for drawing robot
                    JointDiameter = 20/1000;
                    JointLength = 24/1000;
                    
                    % 1. Define Work Space
                    n_angleDiscrete = 3;
                    ang_mouthOpen = 30*pi/180;
                    R_SJ = rot('z',pi/2)*rot('y',alpha);
                    p_SJ = [r_cube;y_cube;z_cube];
                    T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
                    [T_ST,p_ST] = DefineWorkSpace(n_angleDiscrete,ang_mouthOpen,T_SJ);
                    
                    % 3. Analytic Inverse Kinematics / Compute ISO
                    [n_teeth,n_discrete] = size(T_ST);
                    q_IK = zeros([n_joint,size(T_ST)]);
                    eflag = zeros(size(T_ST));
                    ISO_ang_sum = 0;
                    ISO_ang = zeros(size(T_ST));
                    ISO_lin_sum = 0;
                    ISO_lin = zeros(size(T_ST));
                    for ii = 1:n_teeth
                        for jj = 1:n_discrete
                            [q_IK(:,ii,jj),eflag(ii,jj),~] = AnalyticIK(L1,L2,L3,L4,L5,Ltool,S,M_EF,T_ST{ii,jj});
                            if eflag(ii,jj) == 0
                                break
                            else
                                collisionStatus = RobotCheckCollision(S,q_IK(:,ii,jj),n_joint,7, 0.01,...
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
                    
                    % save/display cost function for each iteration
                    %                         cost(i_r,i_z) = ISO_lin_sum*ISO_ang_sum*ISO_lin_min*ISO_ang_min/(L1^3+L2^3+(sqrt(L3^2+L4^2))^3+L5^3);
                    %                         disp([i_L2,i_L3,i_L4,i_L5,i_r,i_z])
                    %                         disp(cost(i_L2,i_L3,i_L4,i_L5,i_r,i_z))
                    cost(i_L1,i_L2,i_L3,i_L4,i_L5) = ISO_lin_sum*ISO_ang_sum*ISO_lin_min*ISO_ang_min/(L1^3+L2^3+(sqrt(L3^2+L4^2))^3+L5^3);
                    disp([i_L1,i_L2,i_L3,i_L4,i_L5])
                    disp(cost(i_L1,i_L2,i_L3,i_L4,i_L5))
                end
            end
        end
    end
end
% end
%     end
% end
% end
toc

%% result
[val, idx] = max(cost,[],'all','linear');
% [idx1, idx2, idx3, idx4,idx5,idx6] = ind2sub( size( cost ), idx );
[idx1, idx2, idx3,idx4,idx5] = ind2sub( size( cost ), idx );
L1 = L2_space(idx1);
L2 = L2_space(idx2);
L3 = L3_space(idx3);
L4 = L4_space(idx4);
L5 = L5_space(idx5);


%save
% save('data/220905_10','L1','L2','L3','L4','L5','alpha','r_cube','y_cube','z_cube','r_space','y_space','z_space','cost');