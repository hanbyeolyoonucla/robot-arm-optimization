function [T_ST,p_ST] = DefineWorkSpace(halfOn, maxillaOn,mandibleOn, occusalCutOn, axialCutOn, m,ang_mouthOpen,T_SJ)

% number of teeth on one jaw
n = 16;

% define default yaw
default_yaw = -60*pi/180;

% number of discretization of the angle
if occusalCutOn == 1 && axialCutOn == 1
    % occusal cut & axial cut
    yaw = linspace(-10*pi/180, 10*pi/180,m);
    roll = linspace(-30*pi/180, 30*pi/180,m);
    pitch = linspace(-10*pi/180, 10*pi/180,m);
elseif occusalCutOn == 1
    % occusal cut
    yaw = linspace(-80*pi/180,30*pi/180,m);
    roll = linspace(-pi/2,pi/2,m);
    pitch = linspace(-5*pi/180, 0*pi/180,m);
elseif axialCutOn == 1
    % axial cut
    yaw = linspace(-80*pi/180,0,m);
    roll = linspace(-pi/6,pi/6,m);
    pitch = linspace(-5*pi/180,0,m);
end

% position of teeth w.r.t. jaw reference frame
p_JT = zeros(3,2*n);
p = zeros(3,2*n);
p_JT(:,1:n) = csvread('toothpos.csv')';

% rotation matrix
R = cell(3*m);
R_JT = rot('x',pi)*rot('y',pi/2); % original orientation
for jj = 1:m
    R{jj} = R_JT*rot('z',default_yaw)*rot('z',yaw(jj));
    R{jj+m} = R_JT*rot('z',default_yaw)*rot('x',roll(jj));
    R{jj+2*m} = R_JT*rot('z',default_yaw)*rot('y',pitch(jj));
end

% define SE(3)
T = cell(2*n,3*m);
for ii = 1:n
    for jj = 1:3*m
        T{ii,jj} = T_SJ*[R{jj} p_JT(:,ii); zeros(1,3) 1];
        p(:,ii) = T{ii,jj}(1:3,4);
    end
end

% rotation matrix for lower jaw
for jj = 1:m
    R{jj} = R_JT*rot('z',-default_yaw)*rot('z',-yaw(jj));
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
        p(:,ii) = T{ii,jj}(1:3,4);
    end
end

% maxilla
if maxillaOn == 1 && mandibleOn == 1
    if halfOn == 1
         % half
         T_ST = T(n/2+1:n*3/2,:);
         p_ST = p(:,n/2+1:n*3/2);
%         % 5,6,7 teeth maxilla & mandible
%         T_ST = T([n/2+5:n/2+7 n/2+10:n/2+12],:);
%         p_ST = p(:,[n/2+5:n/2+7 n/2+10:n/2+12]);
    else
        T_ST = T;
        p_ST = p;
    end    
elseif maxillaOn == 1
    if halfOn == 1
%         T_ST = T(n/2+1:n,:);
%         p_ST = p(:,n/2+1:n);
%         % 5,6,7 teeth maxilla
%         T_ST = T(n/2+5:n/2+7,:);
%         p_ST = p(:,n/2+5:n/2+7);
        % 6 tooth maxilla
        T_ST = T(n/2+6,:);
        p_ST = p(:,n/2+6);
    else
%         T_ST = T(1:n,:);
%         p_ST = p(:,1:n);
        % 6 tooth maxilla
        T_ST = T(n/2+6,:);
        p_ST = p(:,n/2+6);
    end
elseif mandibleOn == 1
    if halfOn == 1
        T_ST = T(n+1:n*3/2,:);
        p_ST = p(:,n+1:n*3/2);
%         % 5,6,7 teeth mandible
%         T_ST = T(n/2+10:n/2+12,:);
%         p_ST = p(:,n/2+10:n/2+12);
    else
        T_ST = T(n+1:end,:);
        p_ST = p(:,n+1:end);
    end
end

end

%% test code
% clc; close all; clear;
% addpath('./functions')
% halfOn = 1; maxillaOn = 1; mandibleOn = 1;
% occusalCutOn = 1; axialCutOn = 1; m = 5;
% ang_mouthOpen= 20*pi/180;
% T_SJ = eye(4);
% 
% [T_ST,p_ST] = DefineWorkSpace(halfOn, maxillaOn,mandibleOn, occusalCutOn, axialCutOn, m,ang_mouthOpen,T_SJ);
% T_ST = reshape(T_ST, 16*15, 1);
% 
% figure()
% plot3(p_ST(1,:), p_ST(2,:), p_ST(3,:), 'r.');
% grid on; hold on;
% for i = 1:length(T_ST)
%     plotTransforms(T_ST{i}(1:3,4)',rotm2quat(T_ST{i}(1:3,1:3)),'FrameSize',0.01)
% end