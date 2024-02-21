clc; clear; close all;
addpath('functions');

occusalCutOn = 1; axialCutOn = 1;
maxillaOn_Mx = 0; mandibleOn_Mx = 0; halfOn_Mx = 1;
maxillaOn_Md = 0; mandibleOn_Md = 0; halfOn_Md = 0;
n_angle = 5;

% constraints
% lb = [0.082 0.100 0.040 0.090 -pi/2 0 -2 -2 -2];
% lb = [0.05 0.05 0.05 0.05 -pi/2 -pi/2 -0.5 0 -0.5 -pi/2 -pi/2 -0.5 0 -0.5];
% ub = [0.3 0.3 0.3 0.3 pi/2 pi/2 0.5 0.5 0.5 pi/2 pi/2 0.5 0.5 0.5];
lb = [0.05 0.05 0.05 0.05 0 0 -0.5 0 -0.5 -0.5 0 -0.5];
ub = [0.3 0.3 0.3 0.3 pi/2 pi/2 0.5 0.5 0.5 0.5 0.5 0.5];
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
nvars = 12;
[x,fval,exitflag,output,scores] = particleswarm(@PerformanceIndexFunction_INDP_v2,nvars,lb,ub,options);
toc
disp(x)
disp(fval)
%%
x = round(x*1000)/1000;
% Robot's DOF
n_joint = 6;
% fixed parameters
L1 = 0.064; L2 = x(1); L3 = x(2); L4 = x(3); L5 = x(4);
alpha_Mx = x(5); beta_Mx = x(6);
x_JS_Mx = x(7); y_JS_Mx = x(8); z_JS_Mx = x(9);
alpha_Md = alpha_Mx; beta_Md = beta_Mx;
x_JS_Md = x(10); y_JS_Md = x(11); z_JS_Md = x(12);

alpha_MxL = -alpha_Mx; beta_MxL = beta_Mx;
x_JS_MxL = x_JS_Mx; y_JS_MxL = -y_JS_Mx; z_JS_MxL = z_JS_Mx;
alpha_MdL = alpha_MxL; beta_MdL = beta_MxL;
x_JS_MdL = x_JS_Md; y_JS_MdL = -y_JS_Md; z_JS_MdL = z_JS_Md;


% alpha_Md = x(10); beta_Md = x(11);
% x_JS_Md = x(12); y_JS_Md = x(13); z_JS_Md = x(14);
Ltool = 0.144; Ltool1 = 0.091; %tool offet
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

% 3. Define Work Space - Mx
R_JS_Mx = rot('z',pi)*rot('y',beta_Mx)*rot('x',alpha_Mx);
p_JS_Mx = [x_JS_Mx;y_JS_Mx;z_JS_Mx];
T_JS_Mx = [R_JS_Mx p_JS_Mx; zeros(1,3) 1];
T_SJ_Mx = inv(T_JS_Mx);
[T_ST_Mx,~] = DefineWorkSpace(0,1,1,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ_Mx);
% 3. Define Work Space - Md
R_JS_Md = rot('z',pi)*rot('y',beta_Md)*rot('x',alpha_Md);
p_JS_Md = [x_JS_Md;y_JS_Md;z_JS_Md];
T_JS_Md = [R_JS_Md p_JS_Md; zeros(1,3) 1];
T_SJ_Md = inv(T_JS_Md);
[T_ST_Md,~] = DefineWorkSpace(0,1,1,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ_Md);
% 3. Define Work Space - Mx
R_JS_MxL = rot('z',pi)*rot('y',beta_MxL)*rot('x',alpha_MxL);
p_JS_MxL = [x_JS_MxL;y_JS_MxL;z_JS_MxL];
T_JS_MxL = [R_JS_MxL p_JS_MxL; zeros(1,3) 1];
T_SJ_MxL = inv(T_JS_MxL);
[T_ST_MxL,~] = DefineWorkSpace(0,1,1,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ_MxL);
% 3. Define Work Space - Md
R_JS_MdL = rot('z',pi)*rot('y',beta_MdL)*rot('x',alpha_MdL);
p_JS_MdL = [x_JS_MdL;y_JS_MdL;z_JS_MdL];
T_JS_MdL = [R_JS_MdL p_JS_MdL; zeros(1,3) 1];
T_SJ_MdL = inv(T_JS_MdL);
[T_ST_MdL,~] = DefineWorkSpace(0,1,1,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ_MdL);

% Solve IK for Optimal WS
[n_teeth,n_discrete] = size(T_ST_Mx);
q_IK_Mx = zeros([n_joint,size(T_ST_Mx)]);
q_IK_Md = zeros([n_joint,size(T_ST_Md)]);
q_IK_MxL = zeros([n_joint,size(T_ST_MxL)]);
q_IK_MdL = zeros([n_joint,size(T_ST_MdL)]);
for ii = 1:n_teeth
    for jj = 1:n_discrete
        [q_IK_Mx(:,ii,jj),~,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_Mx{ii,jj});
        [q_IK_Md(:,ii,jj),~,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_Md{ii,jj});
        [q_IK_MxL(:,ii,jj),~,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_MxL{ii,jj});
        [q_IK_MdL(:,ii,jj),~,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_MdL{ii,jj});
    end
end
q_Meca_Mx = q_IK_Mx;
q_Meca_Mx(5,:,:) = q_Meca_Mx(5,:,:) + pi/2;
q_Meca_Md = q_IK_Md;
q_Meca_Md(5,:,:) = q_Meca_Md(5,:,:) + pi/2;
q_Meca_MxL = q_IK_MxL;
q_Meca_MxL(5,:,:) = q_Meca_MxL(5,:,:) + pi/2;
q_Meca_MdL = q_IK_MdL;
q_Meca_MdL(5,:,:) = q_Meca_MdL(5,:,:) + pi/2;

% Draw WS to stand upright for visualziation
T_SJ = eye(4);
[T_ST,p_ST] = DefineWorkSpace(0,1,1,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% Draw robot in surgical pose
figure()
% q = zeros(6,1);
% q(5) = -pi/2;
q_Mx = q_IK_Mx(:,13,1);
q_Md = q_IK_Md(:,20,1);
q_MxL = q_IK_MxL(:,4,5);
q_MdL = q_IK_MdL(:,29,5);
T_EF_Mx = Draw_Robot_Custom(T_JS_Mx,S,M,q_Mx,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
T_EF_Md = Draw_Robot_Custom(T_JS_Md,S,M,q_Md,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
T_EF_MxL = Draw_Robot_Custom(T_JS_MxL,S,M,q_MxL,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
T_EF_MdL = Draw_Robot_Custom(T_JS_MdL,S,M,q_MdL,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(T_EF_Mx(1:3,4)',rotm2quat(T_EF_Mx(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_EF_Md(1:3,4)',rotm2quat(T_EF_Md(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_EF_MxL(1:3,4)',rotm2quat(T_EF_MxL(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_EF_MdL(1:3,4)',rotm2quat(T_EF_MdL(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_JS_Mx(1:3,4)',rotm2quat(T_JS_Mx(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_JS_Md(1:3,4)',rotm2quat(T_JS_Md(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_JS_MxL(1:3,4)',rotm2quat(T_JS_MxL(1:3,1:3)),'FrameSize',0.05)
plotTransforms(T_JS_MdL(1:3,4)',rotm2quat(T_JS_MdL(1:3,1:3)),'FrameSize',0.05)
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

% mkdir data/240220
save('data/240220/PSO_stiff_single_data','fval','lb','ub','x');