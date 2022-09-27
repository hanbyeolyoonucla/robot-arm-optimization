clc; clear; close all;
addpath('functions');

% fixed parameter
occusalCutOn = 0;
axialCutOn = 1;
maxillaOn = 1;
mandibleOn = 1;
halfOn = 1;
n_angle = 1;

% Meca link length
L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
Lprime = sqrt(L3^2+L4^2);
L5 = 0.070;
Ltool = 0.140;

% GA algorithm
lb = [0 -L2-Lprime 0.1 L1-L2-Lprime-L5-Ltool];
ub = [pi/2 L2+Lprime L2+Lprime L1+L2+Lprime+L5+Ltool];
% rng default
options = optimoptions('ga','OutputFcn',@gaoutfun_MECA,'MaxStallGenerations', 100); %
tic
[x,fval,exitflag,output,population,scores] = ga(@PerformanceIndexFunction_MECA,4,[],[],[],[],lb,ub,[],options);
toc

%% plot result

% Robot's DOF
n_joint = 6;
% fixed parameters
alpha = x(1);
x_cube = x(2);
y_cube = x(3);
z_cube = x(4);
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
[T_ST,p_ST] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% Draw robot in zero position tilted by alpha
figure(3)
q = zeros(6,1);
% q(5) = -pi/2;
T_EF = Draw_Robot_Meca(Svis,Mvis,q,EF_wvis,M_EFvis,n_joint,JointDiameter,JointLength);
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

mkdir data/220926
save('data/220926/GA_MECA_data','halfOn','occusalCutOn','axialCutOn','maxillaOn','mandibleOn','n_angle',...
    'fval','lb','ub','population','scores','x','gapopulationhistory','gascorehistory','gabestscorehistory');
saveas(figure(1),'data/220926/GA_fig1.fig')
saveas(figure(2),'data/220926/GA_fig2.fig')
saveas(figure(3),'data/220926/GA_fig3.fig')