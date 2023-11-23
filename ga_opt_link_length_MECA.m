clc; clear; close all;
addpath('functions');

% fixed parameter
occusalCutOn = 1;
axialCutOn = 1;
maxillaOn = 1;
mandibleOn = 0;
halfOn = 1;
n_angle = 5;

% Meca link length
L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
L5 = 0.070;
Ltool = 0.144;

% GA algorithm
lb = [-pi/2 -pi/2 -1 -1 -1];
ub = [pi/2 pi/2 1 1 1];
% rng default
options = optimoptions('ga','OutputFcn',@gaoutfun_MECA,'PopulationSize',1000);
% options = optimoptions('gamultiobj','PlotFcn',@gaplotpareto);
tic
rng default
[x,fval,exitflag,output,population,scores] = ga(@PerformanceIndexFunction_MECA,5,[],[],[],[],lb,ub,[],options);
% [x,fval,exitflag,output] = gamultiobj(@PerformanceIndexFunction_MECA,4,[],[],[],[],lb,ub,[],options);
toc
%% plot result

% Robot's DOF
n_joint = 6;
% fixed parameters
alpha = x(1);
beta = x(2);
x_cube = x(3);
y_cube = x(4);
z_cube = x(5);
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

% Define Optimal WS
R_SJ1 = rot('z',pi)*rot('y',alpha)*rot('x',beta);
p_SJ1 = [x_cube;y_cube;z_cube];
p_PR = -R_SJ1'*p_SJ1;
T_SJ = [R_SJ1 p_SJ1; zeros(1,3) 1];
[T_ST,p_ST1] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

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

% T visualization : rotate the robot base by alpha for visualization
% Rvis = rot('y',alpha);
% Tvis = [Rvis zeros(3,1); zeros(1,3) 1];
Tvis = inv(T_SJ);
Ad_Tvis = AdjointT(Tvis);
Mvis = cell(n_joint,1);
temp = reshape(Tvis*[M{:}],[4,4,6]);
for i = 1:n_joint
    Mvis{i} = temp(:,:,i);
end
Svis = Ad_Tvis*S;
EF_wvis = Tvis(1:3,1:3)*EF_w;
M_EFvis = Tvis*M_EF;

% Draw WS to stand upright for visualziation
% R_SJ = rot('z',pi);
% p_SJ = Tvis(1:3,1:3)*[x_cube;y_cube;z_cube];
% T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
T_SJ = eye(4);
[T_ST,p_ST] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% Draw robot in zero position tilted by alpha
figure()
% q = zeros(6,1);
q = q_IK(:,end,1);
% q(5) = -pi/2;
T_EF = Draw_Robot_Meca(Svis,Mvis,q,EF_wvis,M_EFvis,Ltool1,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms(Tvis(1:3,4)',rotm2quat(Tvis(1:3,1:3)),'FrameSize',0.05)
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
% plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.02)
xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
grid on

%% save result

mkdir data/231119
save('data/231119/GA_MECA_maxilla_data','halfOn','occusalCutOn','axialCutOn','maxillaOn','mandibleOn','n_angle',...
    'fval','lb','ub','population','scores','x','gapopulationhistory','gascorehistory','gabestscorehistory');
% saveas(figure(1),'data/221008/GA_fig1.fig')
% saveas(figure(2),'data/221008/GA_fig2.fig')
% saveas(figure(3),'data/221008/GA_fig3.fig')