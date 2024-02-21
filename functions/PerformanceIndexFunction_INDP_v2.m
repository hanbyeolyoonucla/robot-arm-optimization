function performance = PerformanceIndexFunction_INDP_v2(X)

% truncate variables to 1e-3
X = round(X*1000)/1000;

% 0. Fixed Parameter
checkCollisionOn = 1; drawCylinderOn = 0;
occusalCutOn = 1; axialCutOn = 1;
maxillaOn_Mx = 0; mandibleOn_Mx = 0; halfOn_Mx = 1;
maxillaOn_Md = 0; mandibleOn_Md = 0; halfOn_Md = 0;
n_angle = 5;
Ltool = 0.144;
Ltool1 = 0.091; %tool offet
alphatool = -90*pi/180;
ang_mouthOpen = 20*pi/180;
weight_m = 0.1; weight_r = 0.1; weight_s = 1;

% 1. Input Varialbes
L1 = 0.064;
L2 = X(1); L3 = X(2); L4 = X(3); L5 = X(4);
alpha_Mx = X(5); beta_Mx = X(6);
x_JS_Mx = X(7); y_JS_Mx = X(8); z_JS_Mx = X(9);
% alpha_Md = X(10); beta_Md = X(11);
% x_JS_Md = X(12); y_JS_Md = X(13); z_JS_Md = X(14);
alpha_Md = alpha_Mx; beta_Md = beta_Mx;
x_JS_Md = X(10); y_JS_Md = X(11); z_JS_Md = X(12);

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

% 3. Define Work Space - Mx
R_JS_Mx = rot('z',pi)*rot('y',beta_Mx)*rot('x',alpha_Mx);
p_JS_Mx = [x_JS_Mx;y_JS_Mx;z_JS_Mx];
T_JS_Mx = [R_JS_Mx p_JS_Mx; zeros(1,3) 1];
T_SJ_Mx = inv(T_JS_Mx);
[T_ST_Mx,~] = DefineWorkSpace(halfOn_Mx,maxillaOn_Mx,mandibleOn_Mx,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ_Mx);
% 3. Define Work Space - Md
R_JS_Md = rot('z',pi)*rot('y',beta_Md)*rot('x',alpha_Md);
p_JS_Md = [x_JS_Md;y_JS_Md;z_JS_Md];
T_JS_Md = [R_JS_Md p_JS_Md; zeros(1,3) 1];
T_SJ_Md = inv(T_JS_Md);
[T_ST_Md,~] = DefineWorkSpace(halfOn_Md,maxillaOn_Md,mandibleOn_Md,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ_Md);

% check if base is in keep out zone
in_keep_out_zone = 0;
% head constraint
head = [-0.1 0.23; -0.17 0.17; -0.29 0.08]; % xmin xmax ymin ymax zmin zmax
body = [-0.31 0; -0.33 0.33;-0.31 0.12]; % xmin xmax ymin ymax zmin zmax
if (all(p_JS_Mx >= head(:,1)) && all(p_JS_Mx <= head(:,2))) || (all(p_JS_Mx >= body(:,1)) && all(p_JS_Mx <= body(:,2)))
    in_keep_out_zone = 1;
end
% on the half side of patient
if p_JS_Mx(2) < 0
    in_keep_out_zone =1;
end
if (all(p_JS_Md >= head(:,1)) && all(p_JS_Md <= head(:,2))) || (all(p_JS_Md >= body(:,1)) && all(p_JS_Md <= body(:,2)))
    in_keep_out_zone = 1;
end
% on the half side of patient
if p_JS_Md(2) < 0
    in_keep_out_zone =1;
end

% 4. Analytic Inverse Kinematics / Check Collision / Compute ISO
[n_teeth,n_discrete] = size(T_ST_Mx);
q_IK_Mx = zeros([n_joint,size(T_ST_Mx)]);
eflag_IK_Mx = 1;
eflag_collision = 1;
q_IK_Md = zeros([n_joint,size(T_ST_Md)]);
eflag_IK_Md = 1;

% Manipulability and Singularity
ISO_ang_sum = 0;
ISO_lin_sum = 0;
ISO_ang_Mx = zeros(size(T_ST_Mx));
ISO_lin_Mx = zeros(size(T_ST_Mx));
ISO_ang_Md = zeros(size(T_ST_Md));
ISO_lin_Md = zeros(size(T_ST_Md));

% Stiffness Normalization
stiffNormalized = stiffnessScore(L1,L2,L3,L4,L5);

if in_keep_out_zone == 0
for ii = 1:n_teeth
    for jj = 1:n_discrete
        [q_IK_Mx(:,ii,jj),eflag_IK_Mx,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_Mx{ii,jj});
        [q_IK_Md(:,ii,jj),eflag_IK_Md,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST_Md{ii,jj});
        % if no IK exits, break
        if eflag_IK_Md == 0 || eflag_IK_Mx == 0
            break
        else % if IK exists
            collisionStatus_Mx = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q_IK_Mx(:,ii,jj),n_joint,7,...
                L1,L2,L3,L4,L5,Ltool,Ltool1,T_SJ_Mx);
            collisionStatus_Md = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q_IK_Md(:,ii,jj),n_joint,7,...
                L1,L2,L3,L4,L5,Ltool,Ltool1,T_SJ_Md);
            if collisionStatus_Md == 0 && collisionStatus_Mx == 0
                [J, ~, ~] = AJacobian(S,q_IK_Mx(:,ii,jj),M_EF,n_joint);
                J_v = J(4:6,:);
                J_w = J(1:3,:);
                SingularValue_lin = svd(J_v);
                SingularValue_ang = svd(J_w);
                ISO_lin_Mx(ii,jj) = SingularValue_lin(end)/SingularValue_lin(1);
                ISO_ang_Mx(ii,jj) = SingularValue_ang(end)/SingularValue_ang(1);
                ISO_lin_sum = ISO_lin_sum + ISO_lin_Mx(ii,jj);
                ISO_ang_sum = ISO_ang_sum + ISO_ang_Mx(ii,jj);
                [J, ~, ~] = AJacobian(S,q_IK_Md(:,ii,jj),M_EF,n_joint);
                J_v = J(4:6,:);
                J_w = J(1:3,:);
                SingularValue_lin = svd(J_v);
                SingularValue_ang = svd(J_w);
                ISO_lin_Md(ii,jj) = SingularValue_lin(end)/SingularValue_lin(1);
                ISO_ang_Md(ii,jj) = SingularValue_ang(end)/SingularValue_ang(1);
                ISO_lin_sum = ISO_lin_sum + ISO_lin_Md(ii,jj);
                ISO_ang_sum = ISO_ang_sum + ISO_ang_Md(ii,jj);
                
            else
                eflag_collision = 0;
                break
            end
        end
    end
    if eflag_IK_Mx == 0 || eflag_IK_Md == 0 ||eflag_collision == 0
        break
    end
end
end
ISO_lin_min = min(min(ISO_lin_Mx,[],'all'),min(ISO_lin_Md,[],'all'));
ISO_ang_min = min(min(ISO_ang_Mx,[],'all'),min(ISO_ang_Md,[],'all'));
ISO_lin_avg = ISO_lin_sum/(n_teeth*n_discrete);
ISO_ang_avg = ISO_ang_sum/(n_teeth*n_discrete);



% Manipulability
manipulability = ISO_lin_avg*ISO_ang_avg;
singularity = ISO_lin_min*ISO_ang_min;

% 5. Evaluate Performance
performance = -(manipulability^weight_m)*(singularity^weight_r)*(stiffNormalized^weight_s)*eflag_IK_Mx*eflag_IK_Md*eflag_collision;

end