function performance = PerformanceIndexFunction_INDP(X)

% 0. Fixed Parameter
checkCollisionOn = 1;
drawCylinderOn = 0;
occusalCutOn = 1;
axialCutOn = 1;
% maxillaOn = 1;
% mandibleOn = 1;
halfOn = 1;
n_angle = 5;
Ltool = 0.140;
Ltool1 = 0.091; %tool offet
alphatool = -77*pi/180;
ang_mouthOpen = 20*pi/180;

% 1. Input Varialbes
L1 = X(1);
L2 = X(2);
L3 = X(3);
L4 = X(4);
L5 = X(5);
alpha_Mx = X(6);
x_MxWS = X(7);
y_MxWS = X(8);
z_MxWS = X(9);
alpha_Md = X(10);
x_MdWS = X(11);
y_MdWS = X(12);
z_MdWS = X(13);

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

% 3. Define Work Space
R_SJ = rot('z',pi)*rot('y',alpha_Mx);
p_SJ = [x_MxWS;y_MxWS;z_MxWS];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST_Mx,~] = DefineWorkSpace(halfOn,1,0,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);
R_SJ = rot('z',pi)*rot('y',alpha_Md);
p_SJ = [x_MdWS;y_MdWS;z_MdWS];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST_Md,~] = DefineWorkSpace(halfOn,0,1,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% 4. Analytic Inverse Kinematics / Check Collision / Compute ISO
stiffNormalized = 0;
[n_teeth,n_discrete] = size(T_ST_Mx);
q_IK_Mx = zeros([n_joint,size(T_ST_Mx)]);
q_IK_Md = zeros([n_joint,size(T_ST_Md)]);
eflag_Mx = zeros(size(T_ST_Mx));
eflag_Md = zeros(size(T_ST_Mx));
ISO_ang_sum = 0;
ISO_ang = zeros(2*n_teeth,n_discrete);
ISO_lin_sum = 0;
ISO_lin = zeros(2*n_teeth,n_discrete);
for ii = 1:n_teeth
    for jj = 1:n_discrete
        [q_IK_Mx(:,ii,jj),eflag_Mx(ii,jj),~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_Mx{ii,jj});
        [q_IK_Md(:,ii,jj),eflag_Md(ii,jj),~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_Md{ii,jj});
        % if no IK exits, break
        if eflag_Mx(ii,jj) == 0 || eflag_Md(ii,jj) == 0
            stiffNormalized = 0;
            collisionStatus_Mx = 1;
            collisionStatus_Md = 1;
            break
        else % if IK exists
            collisionStatus_Mx = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q_IK_Mx(:,ii,jj),n_joint,7,...
                L1,L2,L3,L4,L5,Ltool,Ltool1);
            collisionStatus_Md = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q_IK_Md(:,ii,jj),n_joint,7,...
                L1,L2,L3,L4,L5,Ltool,Ltool1);
            if collisionStatus_Mx == 0 || collisionStatus_Md == 0
                % Maxilla
                [J, ~, ~] = AJacobian(S,q_IK_Mx(:,ii,jj),M_EF,n_joint);
                J_v = J(4:6,:);
                J_w = J(1:3,:);
                SingularValue_lin = svd(J_v);
                SingularValue_ang = svd(J_w);
                ISO_lin(2*ii-1,jj) = SingularValue_lin(end)/SingularValue_lin(1);
                ISO_ang(2*ii-1,jj) = SingularValue_ang(end)/SingularValue_ang(1);
                ISO_lin_sum = ISO_lin_sum + ISO_lin(2*ii-1,jj);
                ISO_ang_sum = ISO_ang_sum + ISO_ang(2*ii-1,jj);
                % Mandible
                [J, ~, ~] = AJacobian(S,q_IK_Md(:,ii,jj),M_EF,n_joint);
                J_v = J(4:6,:);
                J_w = J(1:3,:);
                SingularValue_lin = svd(J_v);
                SingularValue_ang = svd(J_w);
                ISO_lin(2*ii,jj) = SingularValue_lin(end)/SingularValue_lin(1);
                ISO_ang(2*ii,jj) = SingularValue_ang(end)/SingularValue_ang(1);
                ISO_lin_sum = ISO_lin_sum + ISO_lin(2*ii,jj);
                ISO_ang_sum = ISO_ang_sum + ISO_ang(2*ii,jj);
                % Stiffness Normalization
                stiffNormalized = stiffnessScore(L1,L2,L3,L4,L5);
            else
                stiffNormalized = 0;
                break
            end
        end
    end
    if eflag_Mx(ii,jj) == 0 || collisionStatus_Mx == 1 || eflag_Md(ii,jj) == 0 || collisionStatus_Md == 1
        stiffNormalized = 0;
        break
    end
end
ISO_lin_min = min(ISO_lin,[],'all');
ISO_ang_min = min(ISO_ang,[],'all');
ISO_lin_avg = ISO_lin_sum/(2*n_teeth*n_discrete);
ISO_ang_avg = ISO_lin_sum/(2*n_teeth*n_discrete);



% Manipulability
manipulability = ISO_lin_avg*ISO_ang_avg*ISO_lin_min*ISO_ang_min;

% 5. Evaluate Performance
% performance = [-manipulability -stiffNormalized];
performance = -manipulability*stiffNormalized;
% performance = -stiffNormalized;

end