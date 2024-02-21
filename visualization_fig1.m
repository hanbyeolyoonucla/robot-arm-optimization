% clc; close all; clear;
addpath('functions');

% solutions
% x = [0.064	0.226	0.14	0.256	0.09];  % GA OPT
% x = [0.064	0.279	0.213	0.230	0.090];  % PSO OPT
% x = [0.064	0.261	0.211	0.083	0.133];  % GA OPT
% x = [0.064	0.257	0.204	0.098	0.123];  % PSO OPT
% x = [0.064	0.189	0.16	0.116	0.09];  % GA STIFF
% x = [0.064	0.149	0.147	0.153	0.090];  % PSO STIFF

x = [0.135	0.135	0.038	0.12	0.07];  % MECA
% x = [0.064 0.1650    0.0870    0.1570    0.0500]; % single
% x = [0.064 0.1680    0.1250    0.1260    0.0500]; % quadrent
% x = [0.064 0.2078    0.1294    0.209    0.17307]; % frank

% input: x = [l1, l2, l3, l4, l5, ltool]
L1 = x(1); L2 = x(2); L3 = x(3); L4 = x(4); L5 = x(5);

% Robot's DOF
n_joint = 6;
% fixed parameters
Ltool = 0.144;
Ltool1 = 0.091; %tool offet
alphatool = -90*pi/180;
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

% Draw robot in zero position tilted by alpha
figure(3)
q = zeros(6,1);
q(5) = -pi/2;
T_EF = Draw_Robot_Custom(eye(4),S,M,q,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.02)
xlabel('x [m]','FontSize',10);
ylabel('y [m]','FontSize',10);
zlabel('z [m]','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on