function EF_T = Draw_Robot_Meca(T_JS, q,Ltool1,JointDiameter,JointLength)

n_joint = 6;

L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
L5 = 0.070;
Ltool = 0.141;
alphatool = -90*pi/180;

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

joint_T = cell(n_joint,1);
jointafter_p = cell(n_joint,1);
jointafter_w = cell(n_joint,1);
POE = eye(4)*T_JS;

% Screw position and orientation
for i=1:n_joint
    joint_T{i} = POE*M{i};
    jointafter_p{i} = [joint_T{i}(1,4);joint_T{i}(2,4);joint_T{i}(3,4)];
%     jointafter_w{i} = POE(1:3,1:3)*S{i}(1:3,1);
%     POE = POE*ScrewtoSE3(S{i},q(i));
    jointafter_w{i} = POE(1:3,1:3)*S(1:3,i);
    POE = POE*OneScrewtoSE3(S(:,i),q(i));
    RevoluteJoint(jointafter_p{i},JointDiameter,jointafter_w{i},JointLength)
    if i>1
        hold on
        plot3([jointafter_p{i-1}(1) jointafter_p{i}(1)],[jointafter_p{i-1}(2) jointafter_p{i}(2)],[jointafter_p{i-1}(3) jointafter_p{i}(3)],'LineWidth',2,'Color',[0 0 0])
    end
end

% End effector position and orientation
EF_T = POE*M_EF;
joint_T{6} = POE*M{6};
EFafter_p{1} = jointafter_p{6} + joint_T{6}(1:3,1:3)*[Ltool1;0;0];
EFafter_p{2} = [EF_T(1,4);EF_T(2,4);EF_T(3,4)];
EFafter_w = POE(1:3,1:3)*EF_w;
% RayFinder(EFafter_p,RayDiameter,EFafter_w,RayFinderLength)
hold on
% blue lines
plot3([jointafter_p{6}(1) EFafter_p{1}(1)],[jointafter_p{6}(2) EFafter_p{1}(2)],[jointafter_p{6}(3) EFafter_p{1}(3)],'LineWidth',2,'Color',[0 0 1])
plot3([EFafter_p{1}(1) EFafter_p{2}(1)],[EFafter_p{1}(2) EFafter_p{2}(2)],[EFafter_p{1}(3) EFafter_p{2}(3)],'LineWidth',2,'Color',[0 0 1])
RevoluteJoint(EFafter_p{2},10/1000,EFafter_w,JointLength)
end