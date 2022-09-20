clc; clear; close all;
addpath('functions');
Ltool = 0.140;
lb = [0.045 0.082 0.100 0.040 0.090 0 -Inf 0.1 -Inf];
% ub = [Inf Inf Inf Inf Inf pi Inf Inf Inf];
ub = [0.3 0.3 0.3 0.3 0.3 pi Inf Inf Inf];
A = [0 -1 -1 -1 0 0 0 1 0;
    0 -1 -1 -1 0 0 -1 0 0;
    0 -1 -1 -1 0 0 1 0 0;
    1 -1 -1 -1 -1 0 0 0 -1;
    -1 -1 -1 -1 -1 0 0 0 1];
b = [0;0;0;Ltool;Ltool];
tic
[x,fval,exitflag,output,population,scores] = ga(@PerformanceIndexFunction,9,A,b,[],[],lb,ub);
toc
%% save result
occusalCutOn = 0;
axialCutOn = 1;
maxillaOn = 1;
mandibleOn = 1;
n_angle = 5;
save('data/GA_220920','occusalCutOn','axialCutOn','maxillaOn','mandibleOn','n_angle',...
    'fval','lb','ub','population','scores','x');