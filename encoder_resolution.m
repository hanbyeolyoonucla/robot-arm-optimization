clear; close all; clc;

addpath('functions')
% addpath(genpath('data'))
load('data/221010/GA_INDP_data.mat','x')

% MECA Link Lengths
L1 = x(1);
L2 = x(2);
L3 = x(3);
L4 = x(4);
L5 = x(5);
Ltool = 0.140;
Ltool1 = 0.091; %tool offet
alphatool = -77*pi/180;
% Workspace Parameters
halfOn = 1;
maxillaOn = 1;
mandibleOn = 1;
occusalCutOn = 1;
axialCutOn = 1;
n_angle = 5;
ang_mouthOpen = 20*pi/180;
alpha = x(6);
x_WS = x(7);
y_WS = x(8);
z_WS = x(9);
alpha_MD = x(10);
x_WS_MD = x(11);
y_WS_MD = x(12);
z_WS_MD = x(13);

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

% Draw robot in zero position
figure(1)
q = zeros(6,1);
Draw_Robot_Meca(S,M,q,EF_w,M_EF,Ltool1,n_joint,JointDiameter,JointLength);
hold on
plotTransforms(EF_q',rotm2quat(M_EF(1:3,1:3)),'FrameSize',0.05)
plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.05)

xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
xlim([-100,400]/1000);
ylim([-200,200]/1000);
zlim([-150,400]/1000);
grid on

%% 1. Define Work Space
R_SJ = rot('z',pi)*rot('y',alpha);
p_SJ = [x_WS;y_WS;z_WS];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST,p_ST] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);


R_SJ = rot('z',pi)*rot('y',alpha_MD);
p_SJ = [x_WS_MD;y_WS_MD;z_WS_MD];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST_MD,p_ST_MD] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

T_ST(9:16,:) = T_ST_MD(9:16,:);
p_ST(:,9:16) = p_ST_MD(:,9:16);
figure(1)
hold on
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.05)

%% Inverse Kinematics

accuracy = 5e-6;
[n_teeth,n_discrete] = size(T_ST);
q_IK = zeros([n_joint,size(T_ST)]);

resolution_old = 0;
resolution = 1;
while resolution ~= resolution_old
    resolution_old = resolution;
    for ii = 1:n_teeth
        for jj = 1:n_discrete
            [q_IK(:,ii,jj),eflag_IK,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST{ii,jj});
            if eflag_IK == 0
                disp('error')
                break
            else
                % compute analytic jacobian
                [J, ~, ~] = AJacobian(S,q_IK(:,ii,jj),M_EF,n_joint);
                J_v = J(4:6,:);
                SingularValue = svd(J_v);
                maxDisplacement = SingularValue(1)*2*pi/(2^resolution)*sqrt(2)
                if maxDisplacement > accuracy
                    resolution = resolution + 1
                    break
                end
            end
        end
        if maxDisplacement > accuracy
            break
        end
    end
     
end
%display output angle resolution
disp(resolution)
%%
% for ii = 1:6
%     maxEncoderBit(ii) = log2(2*pi/minAngleResolution(ii));
% end
% disp(maxEncoderBit)
% Draw robot in one arbitrary configuration
% figure(2)
% plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
% hold on
% plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
% plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.05)
% T_EF = Draw_Robot_Meca(S,M,q_IK(:,16,1),EF_w,M_EF,n_joint,JointDiameter,JointLength);
% plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
% T_EF = Draw_Robot_Meca(S,M,q_IK(:,32,1),EF_w,M_EF,n_joint,JointDiameter,JointLength);
% plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
% T_EF = Draw_Robot_Meca(S,M,q_IK(:,16,3),EF_w,M_EF,n_joint,JointDiameter,JointLength);
% plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
% T_EF = Draw_Robot_Meca(S,M,q_IK(:,32,3),EF_w,M_EF,n_joint,JointDiameter,JointLength);
% plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
% 
% xlabel('x','FontSize',10);
% ylabel('y','FontSize',10);
% zlabel('z','FontSize',10);
% set(gca,'FontSize',10);
% axis equal;
% view(3)
% xlim([-150,350]/1000);
% ylim([-250,250]/1000);
% zlim([-250,300]/1000);
% grid on
% %% save
% save('data/220906_30','L1','L2','L3','L4','L5','alpha','r_cube','y_cube','z_cube',...
%     'r_space','y_space','z_space','cost');

%%
% collisionStatus = RobotCheckCollision(S,q_IK(:,32,3),n_joint,7,0.01, L1,L2,L3,L4,L5,Ltool,Ltool1)
