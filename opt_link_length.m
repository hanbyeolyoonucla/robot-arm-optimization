clear all
close all
clc

addpath('functions');
tic
n = 10;
m = 4;
cost = zeros(n,n,n,n,m,m);
L2_space = linspace(0.082,0.2,n);
L3_space = linspace(0.100,0.2,n);
L4_space = linspace(0.040,0.2,n);
theta_space = linspace(0,pi/5,m);
L1 = 0.635;
L5 = 0.09;
for i_L2 = 1:n
    L2 = L2_space(i_L2);
    for i_L3 = 1:n
        L3 = L3_space(i_L3);
        for i_L4 = 1:n
            L4 = L4_space(i_L4);
            r_space = linspace(0.01,L2+L4,n);
            z_space = linspace(L1-L2-L4-0.19,L1+L2+L4-0.19,m);
            for i_r = 1:n
                r_cube = r_space(i_r);
                for i_theta = 1:m
                    theta_cube = theta_space(i_theta);
                    for i_z = 1:m
                        z_cube = z_space(i_z);
%% 0. Forward Kinematics
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
joint_q{6} = [L4+L5;0;L1+L2+L3];
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
joint_w{6} = [1;0;0];
for i=1:6
    S(:,i) = [joint_w{i} ; - cross(joint_w{i},joint_q{i},1)];
end

% EF frame's SE(3) when qi = 0
tool_x = 0.120;
tool_z = -0.075;
EF_q = [L4 + L5 + tool_x; 0; L1+L2+L3 + tool_z];
EF_w = rot('y',80*pi/180)*[1;0;0];
M_EF = [rot('y',80*pi/180) EF_q;zeros(1,3) 1];

% Joint shape information for drawing robot
JointDiameter = 20/1000;
JointLength = 24/1000;

% Draw robot in zero position
% figure(1)
% q = zeros(6,1);
% Draw_Robot_Meca(S,M,q,EF_w,M_EF,n_joint,JointDiameter,JointLength)
% xlabel('x','FontSize',10);
% ylabel('y','FontSize',10);
% zlabel('z','FontSize',10);
% set(gca,'FontSize',10);
% axis equal;
% view(3)
% xlim([-150,350]/1000);
% ylim([-250,250]/1000);
% zlim([-300,400]/1000);
% grid on

%% 1. Define Work Space
n_discrete = 2;
x = linspace(-0.05,0.05,n_discrete);
[X,Y,Z] = meshgrid(x);
X = reshape(X,length(X(:,1)),[]);
Y = reshape(Y,length(Y(:,1)),[]);
Z = reshape(Z,length(Z(:,1)),[]);
for i = 1:length(X(:,1))
    for j = 1:length(X(1,:))
        rotated_pt = rot('z',theta_cube)*[X(i,j);Y(i,j);Z(i,j)];
        X(i,j) = r_cube + rotated_pt(1);
        Y(i,j) = rotated_pt(2);
        Z(i,j) = z_cube + rotated_pt(3);
    end
end

%% Inverse Kinematics / Compute ISO
T_desired = M_EF;
R_lower = rot('y',pi)*rot('z',-pi/2-theta_cube)*rot('y',-15*pi/180);
R_upper = rot('y',pi)*rot('z',pi/2-theta_cube)*rot('y',-15*pi/180);
q_iteration = zeros([6,size(X)]);
q_init = zeros(n_joint,1);
ISO_sum = 0;
ISO = zeros(size(X));
for i = 1:length(X(:,1))
    if i <= n_discrete/2
        T_desired(1:3,1:3) = R_lower;
    else
        T_desired(1:3,1:3) = R_upper;
    end
    for j = 1:length(X(1,:))
        T_desired(1:3,4) = [X(i,j); Y(i,j); Z(i,j)];
        [q_iteration(:,i,j), error, iteration] = InverseKinematics_itr(T_desired,S,M_EF,q_init,n_joint);
        if error(iteration-1) < 1e-3
            [J, EulerZYX_init, p] = AJacobian(S,q_iteration(:,i,j),M_EF,n_joint);
            J_v = J(4:6,:);
            SingularValue = svd(J_v);
            ISO(i,j) = SingularValue(end)/SingularValue(1);
            ISO_sum = ISO_sum + ISO(i,j);
        end
    end
end
ISO_min = min(ISO,[],'all');

% Draw robot in one arbitrary configuration
% figure(1)
% Draw_Robot_Meca(S,M,q_iteration(:,1,1),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% Draw_Robot_Meca(S,M,q_iteration(:,end,1),EF_w,M_EF,n_joint,JointDiameter,JointLength)

cost(i_L2,i_L3,i_L4,i_r,i_theta,i_z) = ISO_sum*ISO_min/(L1^3+L2^3+(sqrt(L3^2+L4^2))^3);
disp([i_L2,i_L3,i_L4,i_r,i_theta,i_z])
disp(cost(i_L2,i_L3,i_L4,i_r,i_theta,i_z))
                    end
                end
            end
        end
    end
end
toc
%% validation
[val, idx] = max(cost,[],'all','linear');
[idx1, idx2, idx3, idx4,idx5,idx6] = ind2sub( size( cost ), idx );
L2 = L2_space(idx1);
L3 = L3_space(idx2);
L4 = L4_space(idx3);
r_space = linspace(0.01,L2+L4,n);
z_space = linspace(L1-L2-L4-0.19,L1+L2+L4-0.19,m);
r_cube = r_space(idx4);
theta_cube = theta_space(idx5);
z_cube = z_space(idx6);
% %% plot
% figure(2)
% plot3(X,Y,Z,'r.')
% Draw_Robot_Meca(S,M,q_iteration(:,2,1),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% Draw_Robot_Meca(S,M,q_iteration(:,2,1),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% Draw_Robot_Meca(S,M,q_iteration(:,1,2),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% Draw_Robot_Meca(S,M,q_iteration(:,1,2),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% xlabel('x','FontSize',10);
% ylabel('y','FontSize',10);
% zlabel('z','FontSize',10);
% set(gca,'FontSize',10);
% axis equal;
% view(3)
% xlim([-150,350]/1000);
% ylim([-250,250]/1000);
% zlim([-300,400]/1000);
% grid on
% 
% %% plot
% figure(2)
% plot3(X,Y,Z,'r.')
% xlabel('x','FontSize',10);
% ylabel('y','FontSize',10);
% zlabel('z','FontSize',10);
% set(gca,'FontSize',10);
% axis equal;
% view(3)
% xlim([-150,350]/1000);
% ylim([-250,250]/1000);
% zlim([-300,400]/1000);
% grid on