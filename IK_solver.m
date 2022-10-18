clc; clear; close all;

L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
L5 = 0.070;
Ltool = 0;
Ltool1 = 0; %tool offet
alphatool = -180*pi/180;

% 2. Forward Kinematics
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
M_EF = [rot('y',alphatool) EF_q;zeros(1,3) 1];

% data
data = dlmread('data/cartesian_flange.txt');
data(:,1:3) = data(:,1:3)*0.001;
data(:,4:6) = data(:,4:6)*pi/180;
for ii = 1:length(data)
    R = eul2rotm(data(ii,4:6),'XYZ');
    T = [R data(ii,1:3)'; zeros(1,3) 1];
    [q_IK(:,ii),eflag,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T);
    disp(eflag)
    if eflag == 0
        disp(ii)
        break
    end
end

q_Meca = q_IK';
q_Meca(:,5) = q_Meca(:,5) + pi/2;

writematrix(q_Meca,'IK_MECA.txt','Delimiter','tab')
type 'IK_MECA.txt'