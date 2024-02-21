clc; clear; close all;
addpath('functions');

occusalCutOn = 1; axialCutOn = 1;
maxillaOn = 1; mandibleOn = 1; halfOn = 0;
n_angle = 5;

% constraints
% lb = [0.082 0.100 0.040 0.090 -pi/2 0 -2 -2 -2];
lb = [0.05 0.05 0.05 0.05 -pi/2 0 0 -0.5 -0.5];
ub = [0.3 0.3 0.3 0.3 pi/2 pi/2 0.5 0.5 0.5];
% A = [0 -1 -1 -1 0 0 0 1 0;
%     0 -1 -1 -1 0 0 -1 0 0;
%     0 -1 -1 -1 0 0 1 0 0;
%     1 -1 -1 -1 -1 0 0 0 -1;
%     -1 -1 -1 -1 -1 0 0 0 1];
% b = [0;0;0;Ltool;Ltool];

% options
hybridopts = optimoptions('fmincon','OptimalityTolerance',1e-10);
options = optimoptions('particleswarm','HybridFcn',{'fmincon',hybridopts});
options.SwarmSize = 1000;
options.UseParallel = true;

tic
rng default
nvars = 9;
[x,fval,exitflag,output,scores] = particleswarm(@PerformanceIndexFunction_v2,nvars,lb,ub,options);
toc
disp(x)
disp(fval)
%%
x = round(x*1000)/1000;
% Robot's DOF
n_joint = 6;
% fixed parameters
L1 = 0.064;
L2 = x(1);
L3 = x(2);
L4 = x(3);
L5 = x(4);
alpha = x(5);
beta = x(6);
x_cube = x(7);
y_cube = x(8);
z_cube = x(9);
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
joint_q{4} = [0;0;L1+L2+L3];
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

% Define Optimal WS
R_JS = rot('z',pi)*rot('y',beta)*rot('x',alpha);
p_JS = [x_cube;y_cube;z_cube];
T_JS = [R_JS p_JS; zeros(1,3) 1];
T_SJ = inv(T_JS);
[T_ST,~] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% Solve IK for Optimal WS
[n_teeth,n_discrete] = size(T_ST);
q_IK = zeros([n_joint,size(T_ST)]);
for ii = 1:n_teeth
    for jj = 1:n_discrete
        [q_IK(:,ii,jj),~,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST{ii,jj});
    end
end
q_Meca = q_IK;
q_Meca(5,:,:) = q_Meca(5,:,:) + pi/2;

% Draw WS to stand upright for visualziation
T_SJ = eye(4);
[T_ST,p_ST] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% Draw robot in surgical pose
figure()
% q = zeros(6,1);
% q(5) = -pi/2;
q = q_IK(:,end,1);
T_EF = Draw_Robot_Custom(T_JS,S,M,q,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_JS(1:3,4)',rotm2quat(T_JS(1:3,1:3)),'FrameSize',0.05)
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.02)
xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on

% Draw robot in zero position
figure()
q = zeros(6,1);
q(5) = -pi/2;
T_EF = Draw_Robot_Custom(eye(4),S,M,q,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
hold on
T_EF_Meca = Draw_Robot_Meca(eye(4),q,Ltool1,JointDiameter,JointLength);
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_EF_Meca(1:3,4)',rotm2quat(T_EF_Meca(1:3,1:3)),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.02)
xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on
%% save result

% mkdir data/240221
save('data/240221/PSO_A1','fval','lb','ub','x');