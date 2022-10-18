addpath('functions')
L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
L5 = 0.070;
Ltool = 0.140;
Ltool1 = 0.091; %tool offet
alphatool = -77*pi/180;
ang_mouthOpen = 20*pi/180;
alpha = x(1);
x_cube = x(2);
y_cube = x(3);
z_cube = x(4);

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
Ltool = 0.140;
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
view(0,0)
xlim([-100,400]/1000);
ylim([-200,200]/1000);
zlim([-150,400]/1000);
grid on

%% 1. Define Work Space
n_angleDiscrete = 3;
ang_mouthOpen = 30*pi/180;
R_SJ = rot('z',pi/2)*rot('y',alpha);
% p_SJ = [r_cube;0;z_cube];
p_SJ = [x_cube;y_cube;z_cube];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST,p_ST] = DefineWorkSpace(n_angleDiscrete,ang_mouthOpen,T_SJ);
figure(1)
hold on
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.05)

figure(3)
grid on
hold on
view(3)
axis equal;

% ylim([-50,50]/1000);
xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
for jj = 1:5
    T_temp = T_ST{17,jj};
    p_temp = T_temp (1:3,4);
    R_temp = T_temp (1:3,1:3);
    figure(3)
    plotTransforms(p_temp',rotm2quat(R_temp),'FrameSize',0.01)
end
%% Inverse Kinematics / Compute ISO
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
            [J, ~, ~] = AJacobian(S,q_IK(:,ii,jj),M_EF,n_joint);
            J_v = J(4:6,:);
            J_w = J(1:3,:);
            SingularValue_lin = svd(J_v);
            SingularValue_ang = svd(J_w);
            ISO_lin(ii,jj) = SingularValue_lin(end)/SingularValue_lin(1);
            ISO_ang(ii,jj) = SingularValue_ang(end)/SingularValue_ang(1);
            ISO_lin_sum = ISO_lin_sum + ISO_lin(ii,jj);
            ISO_ang_sum = ISO_ang_sum + ISO_ang(ii,jj);
        end
    end
    if eflag(ii,jj) == 0
        break
    end
end
ISO_lin_min = min(ISO_lin,[],'all');
ISO_ang_min = min(ISO_ang,[],'all');

% save/display cost function for each iteration
opt_cost = ISO_lin_sum*ISO_ang_sum*ISO_lin_min*ISO_ang_min/(L1^3+L2^3+(sqrt(L3^2+L4^2))^3+L5^3);
disp([L1,L2,L3,L4,L5,alpha,r_cube,y_cube,z_cube])
disp(opt_cost)

% Draw robot in one arbitrary configuration
figure(2)
plot3(p_ST(1,:),p_ST(2,:),p_ST(3,:),'r.');
hold on
plotTransforms(p_SJ',rotm2quat(R_SJ),'FrameSize',0.02)
plotTransforms([0,0,0],rotm2quat(eye(3)),'FrameSize',0.05)
T_EF = Draw_Robot_Meca(S,M,q_IK(:,16,1),EF_w,M_EF,n_joint,JointDiameter,JointLength);
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
T_EF = Draw_Robot_Meca(S,M,q_IK(:,32,1),EF_w,M_EF,n_joint,JointDiameter,JointLength);
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
T_EF = Draw_Robot_Meca(S,M,q_IK(:,16,3),EF_w,M_EF,n_joint,JointDiameter,JointLength);
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)
T_EF = Draw_Robot_Meca(S,M,q_IK(:,32,3),EF_w,M_EF,n_joint,JointDiameter,JointLength);
plotTransforms(T_EF(1:3,4)',rotm2quat(T_EF(1:3,1:3)),'FrameSize',0.05)

xlabel('x','FontSize',10);
ylabel('y','FontSize',10);
zlabel('z','FontSize',10);
set(gca,'FontSize',10);
axis equal;
view(3)
xlim([-150,350]/1000);
ylim([-250,250]/1000);
zlim([-250,300]/1000);
grid on
%% save
save('data/220906_30','L1','L2','L3','L4','L5','alpha','r_cube','y_cube','z_cube',...
    'r_space','y_space','z_space','cost');

%%
% collisionStatus = RobotCheckCollision(S,q_IK(:,32,3),n_joint,7,0.01, L1,L2,L3,L4,L5,Ltool,Ltool1)
