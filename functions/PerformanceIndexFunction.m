function performance = PerformanceIndexFunction(X)

% truncate variables to 1e-3
X = round(X*1000)/1000;

% 0. Fixed Parameter
checkCollisionOn = 1;
drawCylinderOn = 0;
occusalCutOn = 1;
axialCutOn = 1;
maxillaOn = 1;
mandibleOn = 1;
halfOn = 1;
n_angle = 5;
Ltool = 0.144;
Ltool1 = 0.091; %tool offet
alphatool = -90*pi/180;
ang_mouthOpen = 20*pi/180;
weight_m = 0.1;
weight_r = 0.1;
weight_s = 1;


% 1. Input Varialbes
L1 = 0.064;
L2 = X(1);
L3 = X(2);
L4 = X(3);
L5 = X(4);
alpha = X(5);
beta = X(6);
x_WS = X(7);
y_WS = X(8);
z_WS = X(9);

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
R_SJ = rot('z',pi)*rot('y',alpha)*rot('x',beta);
p_SJ = [x_WS;y_WS;z_WS];
T_SJ = [R_SJ p_SJ; zeros(1,3) 1];
[T_ST,~] = DefineWorkSpace(halfOn,maxillaOn,mandibleOn,occusalCutOn, axialCutOn,n_angle,ang_mouthOpen,T_SJ);

% check if base is in keep out zone
T_JS = inv(T_SJ);
p_JS = T_JS(1:3,4);
in_keep_out_zone = 0;
% head constraint
head = [-0.1 0.23; -0.17 0.17; -0.29 0.08]; % xmin xmax ymin ymax zmin zmax
body = [-0.31 0; -0.33 0.33;-0.31 0.12]; % xmin xmax ymin ymax zmin zmax
if (all(p_JS >= head(:,1)) && all(p_JS <= head(:,2))) || (all(p_JS >= body(:,1)) && all(p_JS <= body(:,2)))
    in_keep_out_zone = 1;
end

% 4. Analytic Inverse Kinematics / Check Collision / Compute ISO
[n_teeth,n_discrete] = size(T_ST);
q_IK = zeros([n_joint,size(T_ST)]);
eflag_IK = 1;
eflag_collision = 1;

% Manipulability and Singularity
ISO_ang_sum = 0;
ISO_ang = zeros(size(T_ST));
ISO_lin_sum = 0;
ISO_lin = zeros(size(T_ST));

% Stiffness Normalization
stiffNormalized = stiffnessScore(L1,L2,L3,L4,L5);

if in_keep_out_zone == 0
for ii = 1:n_teeth
    for jj = 1:n_discrete
        [q_IK(:,ii,jj),eflag_IK,~] = AnalyticIK(n_joint,L1,L2,L3,L4,L5,Ltool,alphatool,S,M_EF,T_ST{ii,jj});
        % if no IK exits, break
        if eflag_IK == 0
            break
        else % if IK exists
            collisionStatus = RobotCheckCollision(checkCollisionOn,drawCylinderOn,S,q_IK(:,ii,jj),n_joint,7,...
                L1,L2,L3,L4,L5,Ltool,Ltool1);
            if collisionStatus == 0
                [J, ~, ~] = AJacobian(S,q_IK(:,ii,jj),M_EF,n_joint);
                J_v = J(4:6,:);
                J_w = J(1:3,:);
                SingularValue_lin = svd(J_v);
                SingularValue_ang = svd(J_w);
                ISO_lin(ii,jj) = SingularValue_lin(end)/SingularValue_lin(1);
                ISO_ang(ii,jj) = SingularValue_ang(end)/SingularValue_ang(1);
                ISO_lin_sum = ISO_lin_sum + ISO_lin(ii,jj);
                ISO_ang_sum = ISO_ang_sum + ISO_ang(ii,jj);
                
            else
                eflag_collision = 0;
                break
            end
        end
    end
    if eflag_IK == 0 || eflag_collision == 0
        break
    end
end
end
ISO_lin_min = min(ISO_lin,[],'all');
ISO_ang_min = min(ISO_ang,[],'all');
ISO_lin_avg = ISO_lin_sum/(n_teeth*n_discrete);
ISO_ang_avg = ISO_lin_sum/(n_teeth*n_discrete);



% Manipulability
manipulability = ISO_lin_avg*ISO_ang_avg;
singularity = ISO_lin_min*ISO_ang_min;

% 5. Evaluate Performance
performance = -(manipulability^weight_m)*(singularity^weight_r)*(stiffNormalized^weight_s)*eflag_IK*eflag_collision;

end