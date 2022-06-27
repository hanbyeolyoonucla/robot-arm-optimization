clear all
close all
clc

addpath('functions');


% itr = 0;
itr = zeros(1,6);
cost = zeros(20,20,20,10,10,10);
for L2 = 10:10:200
    itr(1) = itr(1) + 1;
    itr(2:6) = [0 0 0 0 0];
    for L3 = 10:10:200
        itr(2) = itr(2) + 1;
        itr(3:6) = [0 0 0 0];
        for L4 = 110:10:300
            itr(3) = itr(3) + 1;
            itr(4:6) = [0 0 0];
            for r_cube = 0.1:0.1:1  % the larger the better
                itr(4) = itr(4) + 1;
                itr(5:6) = [0 0];
                for theta_cube = 0:5*pi/180:45*pi/180 % 0 rad is the best
                    itr(5) = itr(5) + 1;
                    itr(6) = 0;
                    for z_cube = -0.4:0.1:0.5 % the larger the better
                        itr(6) = itr(6) + 1;
%% 0. Forward Kinematics
% Updating Frequency
dt = 0.001;

% Robot's DOF
n_joint = 6;

% Joint positions' SE(3) when qi = 0
L1 = 135; % fixed - should be as short as possible due to cost function - redundant with z_cube variation
% L2 = 135; % 100 < L2 < 200
% L3 = 38; % 0 < L3 < 100
% L4 = 120; % 100 < L4 < 200
% L2 = 110; % 100 < L2 < 200
% L3 = 50; % 0 < L3 < 100
% L4 = 150; % 100 < L4 < 200
L5 = 70;
M = cell(n_joint,1);
joint_q = cell(n_joint,1);
joint_q{1} = [0;0;0];
joint_q{2} = [0;0;L1]/1000;
joint_q{3} = [0;0;L1+L2]/1000;
joint_q{4} = [L4/2;0;L1+L2+L3]/1000;
joint_q{5} = [L4;0;L1+L2+L3]/1000;
joint_q{6} = [L4+L5;0;L1+L2+L3]/1000;
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
tool_x = 120;
tool_z = -75;
EF_q = [L4 + L5 + tool_x; 0; L1+L2+L3 + tool_z]/1000;
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
n_discrete = 6;
x = linspace(-0.05,0.05,n_discrete);
[X,Y,Z] = meshgrid(x);
X = reshape(X,length(X(:,1)),[]);
Y = reshape(Y,length(Y(:,1)),[]);
Z = reshape(Z,length(Z(:,1)),[]);

% r_cube = 0.15; % 0.2 < x_cube < L2 + L4
% theta_cube = 0; % 0 < theta_cube < 90
% z_cube = -0.15; % -(L1+L2+L3) < < (L1+L2+L3)
% r_cube = 0.25; % 0.2 < x_cube < L2 + L4
% theta_cube = 10*pi/180; % 0 < theta_cube < 90
% z_cube = 0.1; % -(L1+L2+L3) < < (L1+L2+L3)
for i = 1:length(X(:,1))
    for j = 1:length(X(1,:))
        rotated_pt = rot('z',theta_cube)*[X(i,j);Y(i,j);Z(i,j)];
        X(i,j) = r_cube + rotated_pt(1);
        Y(i,j) = rotated_pt(2);
        Z(i,j) = z_cube + rotated_pt(3);
    end
end
% figure(1)
plot3(X,Y,Z,'r.')

%% Inverse Kinematics / Compute ISO
T_desired = M_EF;
R_lower = rot('y',pi)*rot('z',-pi/2-theta_cube)*rot('y',-15*pi/180);
R_upper = rot('y',pi)*rot('z',pi/2-theta_cube)*rot('y',-15*pi/180);
q_iteration = zeros([6,size(X)]);
q_init = zeros(n_joint,1);
% q_init = [0; 0; 0; 0; pi/2; pi/2];
ISO_sum = 0;
ISO = zeros(size(X));
for i = 1:length(X(:,1))
%     disp(i)
    if i <= n_discrete/2
        T_desired(1:3,1:3) = R_lower;
    else
        T_desired(1:3,1:3) = R_upper;
    end
    for j = 1:length(X(1,:))
        T_desired(1:3,4) = [X(i,j); Y(i,j); Z(i,j)];
        [q_iteration(:,i,j), error, iteration] = InverseKinematics_itr(T_desired,S,M_EF,q_init,n_joint);
        if error(iteration-1) < 1e-2
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

cost(itr(1),itr(2),itr(3),itr(4),itr(5),itr(6)) = ISO_sum*ISO_min/(L1^3+L2^3+(sqrt(L3^2+L4^2))^3)*1000^3;
disp(itr)
disp(cost(itr(1),itr(2),itr(3),itr(4),itr(5),itr(6)))
                    end
                end
            end
        end
    end
end

%%
% [val, idx] = max(cost,[],'all','linear')
% %% plot
% figure(2)
% plot3(X,Y,Z,'r.')
% Draw_Robot_Meca(S,M,q_iteration(:,2,13),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% Draw_Robot_Meca(S,M,q_iteration(:,2,18),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% Draw_Robot_Meca(S,M,q_iteration(:,5,13),EF_w,M_EF,n_joint,JointDiameter,JointLength)
% Draw_Robot_Meca(S,M,q_iteration(:,5,18),EF_w,M_EF,n_joint,JointDiameter,JointLength)
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