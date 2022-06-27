function [MM,CC,gg, Jt] = RobotDynamicParameters(S_i, M, J, f_iti, q, dqdt, n_torquejoint)

LargeS=zeros(6*n_torquejoint, n_torquejoint);
LargeG=eye(6*n_torquejoint, 6*n_torquejoint);
LargeJ=zeros(6*n_torquejoint, 6*n_torquejoint);
LargeadV=zeros(6*n_torquejoint, 6*n_torquejoint);
Ad_ft=zeros(6*n_torquejoint, 6*n_torquejoint);

f_=cell(n_torquejoint,1);


for i=1:n_torquejoint
    
    % Large S
    LargeS(6*(i-1)+1:6*i,i)=S_i{i};
    
    % Large J
    LargeJ(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=J{i};
    
    % Large G
    f_{i}=M{i}*OneScrewtoSE3(S_i{i},q(i));
    if i>1
        f_ji=f_{i};
        for j=(i-1):-1:1
            LargeG(6*(i-1)+1:6*i,6*(j-1)+1:6*j)=AdjointT(SE3Inverse(f_ji));
            f_ji=f_{j}*f_ji;
        end
    end
    
    % Jt for external force
    Ad_ft(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=AdjointT(SE3Inverse(f_iti{i}));
    
end

% Large V
V=LargeG*LargeS*dqdt;

% Large adV
for i=1:n_torquejoint
    LargeadV(6*(i-1)+1:6*i,6*(i-1)+1:6*i)=adjointV(V(6*(i-1)+1:6*i,1));
end

% P0
P0=zeros(6*n_torquejoint,6);
P0(1:6,1:6)=AdjointT(SE3Inverse(f_{1}));


MM=LargeS'*LargeG'*LargeJ*LargeG*LargeS;
CC=LargeS'*LargeG'*(LargeJ*LargeG*LargeadV-LargeadV'*LargeJ*LargeG)*LargeS;
gg=LargeS'*LargeG'*LargeJ*LargeG*P0*[0;0;0;0;0;-9.8];
Jt=-Ad_ft*LargeG*LargeS;
end