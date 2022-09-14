function [T_ST,p_ST] = DefineWorkSpace(maxillaOn,mandibleOn, m,ang_mouthOpen,T_SJ)

% number of teeth on one jaw
n = 16;

% number of discretization of the angle
alpha = linspace(-80*pi/180,30*pi/180,m);
beta = linspace(-pi/2,pi/2,m);
gamma = linspace(-30*pi/180, 30*pi/180,m);

% position of teeth w.r.t. jaw reference frame
p_JT = zeros(3,2*n);
p_ST = zeros(3,2*n);
p_JT(:,1:n) = csvread('toothpos.csv')';

% rotation matrix
R = cell(3*m);
R_JT = rot('x',pi); % original orientation
for jj = 1:m
    R{jj} = R_JT*rot('x',alpha(jj));
    R{jj+m} = R_JT*rot('z',beta(jj));
    R{jj+2*m} = R_JT*rot('y',gamma(jj));
end

% define SE(3)
T = cell(2*n,3*m);
for ii = 1:n
    for jj = 1:3*m
        T{ii,jj} = T_SJ*[R{jj} p_JT(:,ii); zeros(1,3) 1];
        p_ST(:,ii) = T{ii,jj}(1:3,4);
    end
end

% rotation matrix for lower jaw
for jj = 1:m
    R{jj} = R_JT*rot('x',-alpha(jj));
end

% define screw for mirror the lower jaw
w = rot('y',-ang_mouthOpen/2)*[0;0;1];
q = [0;0;-0.1];
v = -cross(w,q,1);
S = [w;v];
upper2lowerT = OneScrewtoSE3(S,pi);
for ii = n+1:2*n
    for jj = 1:3*m
        T{ii,jj} = T_SJ*upper2lowerT*[R{jj} p_JT(:,ii-n); zeros(1,3) 1];
        p_ST(:,ii) = T{ii,jj}(1:3,4);
    end
end

% maxilla
if maxillaOn == 1 && mandibleOn == 1
    T_ST = T;
elseif maxillaOn == 1
    T_ST = T(1:n,:);
elseif mandibleOn == 1
    T_ST = T(n+1:end,:);
end

end

